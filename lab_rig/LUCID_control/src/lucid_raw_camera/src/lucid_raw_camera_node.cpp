// LUCID Arena camera -> ROS 2 for RAW research capture.
//
// Publishes, per camera namespace:
//   image_raw            Bayer RAW as the camera sends it (8/16-bit as is, 12p unpacked to 16-bit,
//                        24-bit in 32-bit words "32SC1" or packed "bayer_rggb24")
//   image_raw/metadata   lucid_raw_camera/LucidMetadata (chunk values of that frame + session settings)
//   camera_info          every frame (DM_clipGUI counts Hz on it)
//   image_rgb/compressed display-only preview rendered from the RAW frame (raw_preview.hpp)
//
// It follows FLIR_control's flir_spinnaker_camera_node so the same GUI drives both:
//   - every writable GenICam node is a parameter camera.<Node> / stream.<Node> / tl_device.<Node>,
//     applied at startup (enum -> bool -> int -> float -> string, retried while it makes progress),
//     re-registered when an override unlocks more nodes, read back before streaming, and written
//     to the camera immediately when set at runtime;
//   - "Camera acquisition started." once the stream is up (the GUI's ready mark);
//   - frame values come from chunk data; when a chunk is on but unreadable the field is NaN;
//   - the whole node map goes to <nodemap_dump_dir>/<serial>_<time>_nodemap.json at startup and
//     after live changes (docs/raw_capture_noise_calibration.md §4 in FLIR_control).

#include <ArenaApi.h>

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "lucid_raw_camera/msg/lucid_metadata.hpp"

#include "raw_preview.hpp"

// `git describe --always --dirty` of the source tree at build time (build_info.cpp, generated).
extern const char * const kLucidDriverGitDescribe;

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

std::string NormalizeName(const std::string & value)
{
  std::string normalized;
  for (const unsigned char c : value) {
    if (std::isalnum(c) != 0) {
      normalized.push_back(static_cast<char>(std::tolower(c)));
    }
  }
  return normalized;
}

std::string JsonEscape(const std::string & value)
{
  std::string out;
  out.reserve(value.size() + 2U);
  for (const unsigned char c : value) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20U) {
          char buffer[8];
          std::snprintf(buffer, sizeof(buffer), "\\u%04x", c);
          out += buffer;
        } else {
          out.push_back(static_cast<char>(c));
        }
    }
  }
  return out;
}

std::string JsonNumberOrNull(double value)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream text;
  text << std::setprecision(10) << value;
  return text.str();
}

// ---------- GenApi helpers (Arena ships GenICam 3.3 as namespace GenApi) ----------

// GenICam exceptions do not derive from std::exception, so every handler here is catch (...) and
// asks this for the text of whatever is in flight.
std::string CurrentExceptionText()
{
  try {
    throw;
  } catch (const GenICam::GenericException & exception) {
    return exception.GetDescription();
  } catch (const std::exception & exception) {
    return exception.what();
  } catch (...) {
    return "unknown exception";
  }
}

GenApi::INode * FindNode(GenApi::INodeMap * map, const char * name)
{
  return map == nullptr ? nullptr : map->GetNode(name);
}

bool Readable(GenApi::INode * node)
{
  return node != nullptr && GenApi::IsAvailable(node) && GenApi::IsReadable(node);
}

bool Writable(GenApi::INode * node)
{
  return node != nullptr && GenApi::IsAvailable(node) && GenApi::IsWritable(node);
}

std::optional<double> ReadNumber(GenApi::INodeMap * map, std::initializer_list<const char *> names)
{
  for (const char * name : names) {
    try {
      GenApi::INode * node = FindNode(map, name);
      if (!Readable(node)) {
        continue;
      }
      switch (node->GetPrincipalInterfaceType()) {
        case GenApi::intfIFloat:
          return GenApi::CFloatPtr(node)->GetValue();
        case GenApi::intfIInteger:
          return static_cast<double>(GenApi::CIntegerPtr(node)->GetValue());
        case GenApi::intfIBoolean:
          return GenApi::CBooleanPtr(node)->GetValue() ? 1.0 : 0.0;
        default:
          break;
      }
    } catch (const GenICam::GenericException &) {
    }
  }
  return std::nullopt;
}

std::optional<std::int64_t> ReadInteger(GenApi::INodeMap * map, const char * name)
{
  try {
    GenApi::INode * node = FindNode(map, name);
    if (Readable(node) && node->GetPrincipalInterfaceType() == GenApi::intfIInteger) {
      return GenApi::CIntegerPtr(node)->GetValue();
    }
  } catch (const GenICam::GenericException &) {
  }
  return std::nullopt;
}

std::optional<bool> ReadBool(GenApi::INodeMap * map, std::initializer_list<const char *> names)
{
  for (const char * name : names) {
    try {
      GenApi::INode * node = FindNode(map, name);
      if (Readable(node) && node->GetPrincipalInterfaceType() == GenApi::intfIBoolean) {
        return GenApi::CBooleanPtr(node)->GetValue();
      }
    } catch (const GenICam::GenericException &) {
    }
  }
  return std::nullopt;
}

std::optional<std::string> ReadText(GenApi::INodeMap * map, std::initializer_list<const char *> names)
{
  for (const char * name : names) {
    try {
      GenApi::INode * node = FindNode(map, name);
      if (Readable(node)) {
        return std::string(GenApi::CValuePtr(node)->ToString().c_str());
      }
    } catch (const GenICam::GenericException &) {
    }
  }
  return std::nullopt;
}

bool SetEnumeration(GenApi::INodeMap * map, const char * name, const std::string & entry_name)
{
  GenApi::INode * node = FindNode(map, name);
  if (!Writable(node)) {
    return false;
  }
  GenApi::CEnumerationPtr enumeration(node);
  GenApi::CEnumEntryPtr entry = enumeration->GetEntryByName(entry_name.c_str());
  if (!entry.IsValid() || !GenApi::IsAvailable(entry) || !GenApi::IsReadable(entry)) {
    return false;
  }
  enumeration->SetIntValue(entry->GetValue());
  return true;
}

bool SetBoolean(GenApi::INodeMap * map, const char * name, bool value)
{
  GenApi::INode * node = FindNode(map, name);
  if (!Writable(node)) {
    return false;
  }
  GenApi::CBooleanPtr(node)->SetValue(value);
  return true;
}

bool SetInteger(GenApi::INodeMap * map, const char * name, std::int64_t value)
{
  GenApi::INode * node = FindNode(map, name);
  if (!Writable(node)) {
    return false;
  }
  GenApi::CIntegerPtr(node)->SetValue(value);
  return true;
}

// ---------- pixel formats ----------

struct RawFormat
{
  std::string name;   // PixelFormat symbolic
  int bits{0};        // bits per pixel on the wire
  bool packed12{false};
  std::string cfa;    // "" = mono
};

std::optional<RawFormat> ParseRawFormat(const std::string & name)
{
  RawFormat format;
  format.name = name;
  std::string rest;
  if (name.rfind("Bayer", 0) == 0 && name.size() > 7U) {
    const std::string pair = name.substr(5, 2);
    if (pair == "RG") {
      format.cfa = "RGGB";
    } else if (pair == "GR") {
      format.cfa = "GRBG";
    } else if (pair == "GB") {
      format.cfa = "GBRG";
    } else if (pair == "BG") {
      format.cfa = "BGGR";
    } else {
      return std::nullopt;
    }
    rest = name.substr(7);
  } else if (name.rfind("Mono", 0) == 0) {
    rest = name.substr(4);
  } else {
    return std::nullopt;
  }
  if (rest == "8") {
    format.bits = 8;
  } else if (rest == "12p") {
    format.bits = 12;
    format.packed12 = true;
  } else if (rest == "16") {
    format.bits = 16;
  } else if (rest == "24") {
    format.bits = 24;
  } else {
    return std::nullopt;
  }
  return format;
}

std::string Encoding16(const std::string & cfa)
{
  namespace enc = sensor_msgs::image_encodings;
  if (cfa == "RGGB") {return enc::BAYER_RGGB16;}
  if (cfa == "GRBG") {return enc::BAYER_GRBG16;}
  if (cfa == "GBRG") {return enc::BAYER_GBRG16;}
  if (cfa == "BGGR") {return enc::BAYER_BGGR16;}
  return enc::MONO16;
}

std::string Encoding8(const std::string & cfa)
{
  namespace enc = sensor_msgs::image_encodings;
  if (cfa == "RGGB") {return enc::BAYER_RGGB8;}
  if (cfa == "GRBG") {return enc::BAYER_GRBG8;}
  if (cfa == "GBRG") {return enc::BAYER_GBRG8;}
  if (cfa == "BGGR") {return enc::BAYER_BGGR8;}
  return enc::MONO8;
}

// GenICam 12p: two pixels in three bytes, LSB first (PFNC).
void Unpack12p(const std::uint8_t * src, std::uint16_t * dst, std::size_t pixels, int shift)
{
  const std::size_t pairs = pixels / 2U;
  for (std::size_t i = 0; i < pairs; ++i) {
    const std::uint32_t b0 = src[3U * i];
    const std::uint32_t b1 = src[3U * i + 1U];
    const std::uint32_t b2 = src[3U * i + 2U];
    dst[2U * i] = static_cast<std::uint16_t>(((b0 | ((b1 & 0x0FU) << 8U)) << shift) & 0xFFFFU);
    dst[2U * i + 1U] = static_cast<std::uint16_t>((((b1 >> 4U) | (b2 << 4U)) << shift) & 0xFFFFU);
  }
  if (pixels % 2U != 0U) {
    const std::uint32_t b0 = src[3U * pairs];
    const std::uint32_t b1 = src[3U * pairs + 1U];
    dst[pixels - 1U] = static_cast<std::uint16_t>(((b0 | ((b1 & 0x0FU) << 8U)) << shift) & 0xFFFFU);
  }
}

void Expand24(const std::uint8_t * src, std::uint32_t * dst, std::size_t pixels, bool big_endian)
{
  if (big_endian) {
    for (std::size_t i = 0; i < pixels; ++i) {
      dst[i] = (static_cast<std::uint32_t>(src[3U * i]) << 16U) |
        (static_cast<std::uint32_t>(src[3U * i + 1U]) << 8U) | src[3U * i + 2U];
    }
  } else {
    for (std::size_t i = 0; i < pixels; ++i) {
      dst[i] = src[3U * i] | (static_cast<std::uint32_t>(src[3U * i + 1U]) << 8U) |
        (static_cast<std::uint32_t>(src[3U * i + 2U]) << 16U);
    }
  }
}

bool IsHostBigEndian()
{
  constexpr std::uint16_t probe = 0x0102;
  return reinterpret_cast<const std::uint8_t *>(&probe)[0] == 0x01;
}

enum class ControlMapKind { Camera, Stream, TlDevice };
enum class ControlValueKind { Boolean, Integer, Float, Enumeration, String };

struct ControlBinding
{
  ControlMapKind map_kind;
  ControlValueKind value_kind;
  std::string node_name;
};

const char * ControlPrefix(ControlMapKind kind)
{
  switch (kind) {
    case ControlMapKind::Camera: return "camera";
    case ControlMapKind::Stream: return "stream";
    case ControlMapKind::TlDevice: return "tl_device";
  }
  return "camera";
}

}  // namespace

class LucidRawCameraNode : public rclcpp::Node
{
public:
  LucidRawCameraNode()
  : Node("lucid_camera")
  {
    camera_serial_ = declare_parameter<std::string>("camera_serial", "");
    camera_index_ = declare_parameter<int>("camera_index", 0);
    frame_id_ = declare_parameter<std::string>("frame_id", "lucid_camera_optical_frame");
    acquisition_timeout_ms_ = declare_parameter<int>("acquisition_timeout_ms", 10000);
    device_discovery_timeout_ms_ = declare_parameter<int>("device_discovery_timeout_ms", 2000);
    camera_init_max_attempts_ = std::max(1, static_cast<int>(declare_parameter<int>("camera_init.max_attempts", 6)));
    camera_init_retry_delay_ms_ = std::max(0, static_cast<int>(declare_parameter<int>("camera_init.retry_delay_ms", 2000)));
    publisher_qos_reliability_ = declare_parameter<std::string>("publisher_qos_reliability", "reliable");
    publisher_qos_depth_ = declare_parameter<int>("publisher_qos_depth", 5);
    publish_raw_ = declare_parameter<bool>("publish_raw", true);
    publish_camera_info_ = declare_parameter<bool>("publish_camera_info", true);
    publish_metadata_ = declare_parameter<bool>("publish_metadata", true);
    publish_rgb_compressed_ = declare_parameter<bool>("publish_rgb_compressed", true);
    pixel_format_ = declare_parameter<std::string>("pixel_format", "BayerRG24");
    raw24_container_ = NormalizeName(declare_parameter<std::string>("raw24_container", "uint32")) == "packed" ?
      "packed" : "uint32";
    raw12p_shift_ = std::clamp(static_cast<int>(declare_parameter<std::int64_t>("raw12p_shift", 0)), 0, 4);
    buffer_handling_mode_ = declare_parameter<std::string>("buffer_handling_mode", "OldestFirst");
    stream_buffer_count_ = std::max(2, static_cast<int>(declare_parameter<std::int64_t>("stream_buffer_count", 10)));
    chunk_data_enable_ = declare_parameter<bool>("chunk_data.enable", true);
    chunk_data_selectors_ = declare_parameter<std::vector<std::string>>(
      "chunk_data.selectors",
      std::vector<std::string>{"CRC", "Timestamp", "ExposureTime", "Gain", "BlackLevel", "PixelFormat"});
    timestamp_mode_ = NormalizeName(declare_parameter<std::string>("timestamp.mode", "camera_latched")) == "host" ?
      "host" : "camera_latched";
    timestamp_exposure_latch_ =
      NormalizeName(declare_parameter<std::string>("timestamp.exposure_latch", "start")) == "end" ? "end" : "start";
    timestamp_latch_interval_sec_ = declare_parameter<double>("timestamp.latch_interval_sec", 2.0);
    stream_stats_interval_sec_ = declare_parameter<double>("stream_stats.interval_sec", 30.0);
    // Live changes of nodes that are locked while streaming (HDROutput, AcquisitionFrameRate, …):
    // stop the stream, write, restart (~0.5 s without frames) instead of refusing.
    restart_for_locked_ = declare_parameter<bool>("live.restart_stream_for_locked", true);
    // ExposureTime longer than the frame period lowers AcquisitionFrameRate to fit (the parameter
    // camera.AcquisitionFrameRate becomes the cap), shorter raises it back up to the cap.
    auto_frame_rate_ = declare_parameter<bool>("exposure.auto_frame_rate", false);
    frame_rate_margin_us_ = declare_parameter<double>("exposure.frame_rate_margin_us", 500.0);
    // trigger.mode: "off" = free run, "action" = one frame per GigE Vision action command
    // (TriggerSource Action0, no PTP needed) — tools/bracket.py fires one broadcast command for both cameras.
    trigger_mode_ = NormalizeName(declare_parameter<std::string>("trigger.mode", "off")) == "action" ? "action" : "off";
    trigger_device_key_ = declare_parameter<std::int64_t>("trigger.action.device_key", 0x4A53);
    trigger_group_key_ = declare_parameter<std::int64_t>("trigger.action.group_key", 1);
    trigger_group_mask_ = declare_parameter<std::int64_t>("trigger.action.group_mask", 1);

    raw_balance_ratio_red_ = declare_parameter<double>("raw.balance_ratio_red", kNaN);
    raw_balance_ratio_green_ = declare_parameter<double>("raw.balance_ratio_green", kNaN);
    raw_balance_ratio_blue_ = declare_parameter<double>("raw.balance_ratio_blue", kNaN);
    raw_black_level_raw_ = declare_parameter<std::int64_t>("raw.black_level_raw", -1);
    // -1 = auto (the known clip point of a TRI054S channel output, see EffectiveWhiteLevel), 0 = unknown
    raw_white_level_param_ = declare_parameter<std::int64_t>("raw.white_level_dn", -1);
    nodemap_dump_dir_ = declare_parameter<std::string>("nodemap_dump_dir", "");
    profile_path_ = declare_parameter<std::string>("profile_path", "");
    lens_model_ = declare_parameter<std::string>("lens.model", "");
    lens_f_number_ = declare_parameter<double>("lens.f_number", kNaN);
    temperature_interval_sec_ = declare_parameter<double>("temperature.interval_sec", 1.0);

    force_ip_enable_ = declare_parameter<bool>("network.force_ip.enable", false);
    force_ip_address_ = declare_parameter<std::string>("network.force_ip.address", "");
    force_ip_subnet_mask_ = declare_parameter<std::string>("network.force_ip.subnet_mask", "255.255.255.0");
    force_ip_gateway_ = declare_parameter<std::string>("network.force_ip.gateway", "0.0.0.0");

    camera_info_distortion_model_ = declare_parameter<std::string>("camera_info.distortion_model", "plumb_bob");
    camera_info_d_ = declare_parameter<std::vector<double>>("camera_info.d", std::vector<double>{0, 0, 0, 0, 0});
    const auto k = declare_parameter<std::vector<double>>("camera_info.k", std::vector<double>(9, 0.0));
    const auto r = declare_parameter<std::vector<double>>("camera_info.r", std::vector<double>(9, 0.0));
    const auto p = declare_parameter<std::vector<double>>("camera_info.p", std::vector<double>(12, 0.0));
    if (k.size() != 9U || r.size() != 9U || p.size() != 12U) {
      throw std::runtime_error("camera_info.k/r must have 9 values and camera_info.p 12.");
    }
    std::copy(k.begin(), k.end(), camera_info_k_.begin());
    std::copy(r.begin(), r.end(), camera_info_r_.begin());
    std::copy(p.begin(), p.end(), camera_info_p_.begin());

    // image_rgb/compressed is always the RAW preview here (there is no camera-ISP path).
    if (NormalizeName(declare_parameter<std::string>("rgb_compressed_mode", "preview")) != "preview") {
      RCLCPP_WARN(get_logger(), "rgb_compressed_mode: only 'preview' is implemented; using it.");
    }
    raw_preview::Settings settings;
    settings.max_rate_hz = declare_parameter<double>("preview.max_rate_hz", settings.max_rate_hz);
    settings.downscale = static_cast<int>(declare_parameter<std::int64_t>("preview.downscale", 2));
    settings.gain = declare_parameter<double>("preview.gain", settings.gain);
    settings.target_mean = declare_parameter<double>("preview.target_mean", settings.target_mean);
    settings.max_gain = declare_parameter<double>("preview.max_gain", 1.0e6);
    settings.noise_ceiling = declare_parameter<double>("preview.noise_ceiling", settings.noise_ceiling);
    settings.gamma = declare_parameter<double>("preview.gamma", settings.gamma);
    settings.gray_world =
      NormalizeName(declare_parameter<std::string>("preview.white_balance", "gray_world")) != "none";
    settings.black_level_dn = declare_parameter<double>("preview.black_level_dn", settings.black_level_dn);
    settings.jpeg_quality = static_cast<int>(declare_parameter<std::int64_t>("preview.jpeg_quality", 80));
    preview_settings_ = settings;

    if (!publish_raw_ && !publish_camera_info_ && !publish_metadata_ && !publish_rgb_compressed_) {
      throw std::runtime_error("At least one of publish_raw/camera_info/metadata/rgb_compressed must be true.");
    }
    if (!ParseRawFormat(pixel_format_)) {
      throw std::runtime_error(
        "pixel_format '" + pixel_format_ + "' is not a Bayer/Mono 8, 12p, 16 or 24 format.");
    }

    rclcpp::QoS qos{rclcpp::KeepLast(static_cast<std::size_t>(std::max(1, publisher_qos_depth_)))};
    qos.durability_volatile();
    if (NormalizeName(publisher_qos_reliability_) == "besteffort") {
      qos.best_effort();
    } else {
      qos.reliable();
    }
    if (publish_raw_) {
      raw_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", qos);
    }
    if (publish_camera_info_) {
      camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);
    }
    if (publish_metadata_) {
      metadata_pub_ = create_publisher<lucid_raw_camera::msg::LucidMetadata>("image_raw/metadata", qos);
    }
    if (publish_rgb_compressed_) {
      preview_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("image_rgb/compressed", qos);
    }

    try {
      // A camera another process just let go of stays reserved until its heartbeat expires (6 s on
      // the TRI054S), and a sibling launch may still hold it: retry access-denied for a while.
      for (int attempt = 1;; ++attempt) {
        try {
          InitializeCamera();
          break;
        } catch (...) {
          const std::string text = CurrentExceptionText();
          const bool busy = text.find("ACCESS_DENIED") != std::string::npos ||
            text.find("-1005") != std::string::npos;
          if (!busy || attempt >= camera_init_max_attempts_) {
            throw;
          }
          RCLCPP_WARN(
            get_logger(), "Camera is held by another controller (attempt %d/%d); retrying in %d ms: %s",
            attempt, camera_init_max_attempts_, camera_init_retry_delay_ms_, text.c_str());
          Shutdown();
          std::this_thread::sleep_for(std::chrono::milliseconds(camera_init_retry_delay_ms_));
        }
      }
      running_.store(true);
      if (publish_rgb_compressed_) {
        preview_thread_ = std::thread(&LucidRawCameraNode::PreviewLoop, this);
      }
      if (timestamp_mode_ == "camera_latched") {
        SampleCameraClock();   // the first frame must already have a mapping
        clock_thread_ = std::thread(&LucidRawCameraNode::ClockSyncLoop, this);
      }
      acquisition_thread_ = std::thread(&LucidRawCameraNode::AcquisitionLoop, this);
      housekeeping_timer_ = create_wall_timer(
        std::chrono::milliseconds(250), std::bind(&LucidRawCameraNode::Housekeeping, this));
    } catch (...) {
      Shutdown();
      throw;
    }
  }

  ~LucidRawCameraNode() override
  {
    Shutdown();
  }

private:
  // ======================================================================================
  // Camera bring-up
  // ======================================================================================

  void InitializeCamera()
  {
    system_ = Arena::OpenSystem();
    system_->UpdateDevices(static_cast<std::uint64_t>(std::max(100, device_discovery_timeout_ms_)));
    std::vector<Arena::DeviceInfo> devices = system_->GetDevices();
    if (devices.empty()) {
      throw std::runtime_error("No LUCID camera found (check the NIC, the camera IP and power).");
    }
    std::optional<Arena::DeviceInfo> chosen;
    for (std::size_t i = 0; i < devices.size(); ++i) {
      auto & info = devices[i];
      RCLCPP_INFO(
        get_logger(), "Detected camera[%zu]: vendor='%s' model='%s' serial='%s' ip=%s", i,
        info.VendorName().c_str(), info.ModelName().c_str(), info.SerialNumber().c_str(),
        info.IpAddressStr().c_str());
      if (camera_serial_.empty() ? static_cast<int>(i) == camera_index_ :
        std::string(info.SerialNumber().c_str()) == camera_serial_)
      {
        chosen = info;
      }
    }
    if (!chosen) {
      throw std::runtime_error(
        "camera_serial '" + camera_serial_ + "' was not found among " + std::to_string(devices.size()) +
        " camera(s).");
    }
    if (force_ip_enable_ && !force_ip_address_.empty() &&
      std::string(chosen->IpAddressStr().c_str()) != force_ip_address_)
    {
      RCLCPP_INFO(
        get_logger(), "ForceIP %s -> %s", chosen->IpAddressStr().c_str(), force_ip_address_.c_str());
      system_->ForceIp(
        chosen->MacAddressStr().c_str(), force_ip_address_.c_str(), force_ip_subnet_mask_.c_str(),
        force_ip_gateway_.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));
      system_->UpdateDevices(2000);
      chosen.reset();
      for (auto & info : system_->GetDevices()) {
        if (std::string(info.SerialNumber().c_str()) == camera_serial_) {
          chosen = info;
        }
      }
      if (!chosen) {
        throw std::runtime_error("camera did not come back after ForceIP");
      }
    }

    device_ = system_->CreateDevice(*chosen);
    camera_map_ = device_->GetNodeMap();
    stream_map_ = device_->GetTLStreamNodeMap();
    tl_device_map_ = device_->GetTLDeviceNodeMap();

    if (!buffer_handling_mode_.empty() &&
      !SetEnumeration(stream_map_, "StreamBufferHandlingMode", buffer_handling_mode_))
    {
      RCLCPP_WARN(get_logger(), "Could not set StreamBufferHandlingMode to '%s'.", buffer_handling_mode_.c_str());
    }
    ApplyPixelFormat();
    if (!SetEnumeration(camera_map_, "AcquisitionMode", "Continuous")) {
      throw std::runtime_error("Failed to set AcquisitionMode to Continuous.");
    }
    InitializeControlParameters();
    ApplyPixelFormat();   // HDROutput and other overrides may have reset it
    ApplyChunkData();
    VerifyControlOverridesTookEffect();
    ApplyRawSelectorParameters(true);
    ApplyTriggerMode();
    LogLinkSettings();
    RefreshSessionInfo();
    DumpNodeMap("startup");

    const std::string serial = ReadText(tl_device_map_, {"DeviceSerialNumber"}).value_or("?");
    const std::string model = ReadText(tl_device_map_, {"DeviceModelName"}).value_or("?");
    RCLCPP_INFO(get_logger(), "Using camera model='%s' serial='%s'", model.c_str(), serial.c_str());
    device_->StartStream(static_cast<std::size_t>(stream_buffer_count_));
    stream_started_ = true;
    FindNodesLockedWhileStreaming();
    RCLCPP_INFO(get_logger(), "Camera acquisition started.");
  }

  // trigger.mode. Call with the stream stopped (or not started yet): TriggerMode / TriggerSource are
  // locked while streaming. ActionPTPSyncRequired is cleared so a command acts on arrival without PTP.
  void ApplyTriggerMode()
  {
    trigger_armed_.store(false);
    SetEnumeration(camera_map_, "TriggerSelector", "FrameStart");
    if (!SetEnumeration(camera_map_, "TriggerMode", "Off")) {
      throw std::runtime_error("trigger.mode: TriggerMode is not writable");
    }
    // Low-latency trigger where the camera has it. The TRI054S does not (TriggerLatency is not implemented,
    // capability bit 0), so this is a no-op there: its exposure starts 2/fps − ExposureTime after the
    // command (TriggerLatencyTime; ~1 s at 2 Hz), and the first triggered frame after an ExposureTime change
    // still has the old exposure while its chunk shows the new one (measured 2026-10-20). tools/bracket.py
    // fires this camera first and drops one frame after each exposure change.
    SetEnumeration(camera_map_, "TriggerLatency", trigger_mode_ == "action" ? "OneLine" : "Off");
    if (trigger_mode_ != "action") {
      RCLCPP_INFO(get_logger(), "Trigger: off (free run).");
      return;
    }
    SetInteger(camera_map_, "ActionSelector", 0);
    if (!SetInteger(camera_map_, "ActionDeviceKey", trigger_device_key_) ||
      !SetInteger(camera_map_, "ActionGroupKey", trigger_group_key_) ||
      !SetInteger(camera_map_, "ActionGroupMask", trigger_group_mask_))
    {
      throw std::runtime_error("trigger.mode action: could not write ActionDeviceKey / GroupKey / GroupMask");
    }
    SetBoolean(camera_map_, "ActionPTPSyncRequired", false);
    SetEnumeration(camera_map_, "ActionUnconditionalMode", "On");
    if (ReadBool(camera_map_, {"ActionPTPSyncRequired"}).value_or(false)) {
      RCLCPP_WARN(get_logger(), "ActionPTPSyncRequired stays true — action commands need PTP on this camera.");
    }
    if (!SetEnumeration(camera_map_, "TriggerSource", "Action0") ||
      !SetEnumeration(camera_map_, "TriggerMode", "On"))
    {
      throw std::runtime_error("trigger.mode action: the camera refused TriggerSource Action0 / TriggerMode On");
    }
    trigger_armed_.store(true);
    RCLCPP_INFO(
      get_logger(), "Trigger: action command (Action0, device key 0x%llX, group key %lld, mask 0x%llX), "
      "TriggerLatency %s, TriggerLatencyTime %.0f us.",
      static_cast<unsigned long long>(trigger_device_key_), static_cast<long long>(trigger_group_key_),
      static_cast<unsigned long long>(trigger_group_mask_),
      ReadText(camera_map_, {"TriggerLatency"}).value_or("?").c_str(),
      ReadNumber(camera_map_, {"TriggerLatencyTime"}).value_or(-1.0));
  }

  void ApplyPixelFormat()
  {
    if (ReadText(camera_map_, {"PixelFormat"}).value_or("") == pixel_format_) {
      return;
    }
    if (!SetEnumeration(camera_map_, "PixelFormat", pixel_format_)) {
      throw std::runtime_error("Failed to set PixelFormat to " + pixel_format_);
    }
    RCLCPP_INFO(get_logger(), "PixelFormat set to '%s'", pixel_format_.c_str());
  }

  // ======================================================================================
  // camera.* / stream.* / tl_device.* bridge (same rules as flir_spinnaker_camera_node)
  // ======================================================================================

  GenApi::INodeMap * MapOf(ControlMapKind kind) const
  {
    switch (kind) {
      case ControlMapKind::Camera: return camera_map_;
      case ControlMapKind::Stream: return stream_map_;
      case ControlMapKind::TlDevice: return tl_device_map_;
    }
    return camera_map_;
  }

  // Set by this node's own logic, never through camera.*.
  static bool IsManagedControlNode(ControlMapKind kind, const std::string & name)
  {
    if (kind == ControlMapKind::Camera) {
      return name == "PixelFormat" || name == "AcquisitionMode" || name.rfind("Chunk", 0) == 0;
    }
    if (kind == ControlMapKind::Stream) {
      return name == "StreamBufferHandlingMode";
    }
    return false;
  }

  static std::optional<ControlValueKind> ValueKindOf(GenApi::INode * node)
  {
    switch (node->GetPrincipalInterfaceType()) {
      case GenApi::intfIBoolean: return ControlValueKind::Boolean;
      case GenApi::intfIInteger: return ControlValueKind::Integer;
      case GenApi::intfIFloat: return ControlValueKind::Float;
      case GenApi::intfIEnumeration: return ControlValueKind::Enumeration;
      case GenApi::intfIString: return ControlValueKind::String;
      default: return std::nullopt;
    }
  }

  static int Priority(ControlValueKind kind)
  {
    switch (kind) {
      case ControlValueKind::Enumeration: return 0;
      case ControlValueKind::Boolean: return 1;
      case ControlValueKind::Integer: return 2;
      case ControlValueKind::Float: return 3;
      case ControlValueKind::String: return 4;
    }
    return 10;
  }

  rcl_interfaces::msg::ParameterDescriptor DescriptorFor(GenApi::INode * node, ControlValueKind kind)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    std::ostringstream text;
    text << "GenICam node '" << node->GetName().c_str() << "'";
    const std::string display = node->GetDisplayName().c_str();
    if (!display.empty() && display != node->GetName().c_str()) {
      text << " (" << display << ")";
    }
    const std::string tooltip = node->GetToolTip().c_str();
    if (!tooltip.empty()) {
      text << ". " << tooltip;
    }
    if (kind == ControlValueKind::Enumeration) {
      GenApi::CEnumerationPtr enumeration(node);
      GenApi::StringList_t symbolics;
      enumeration->GetSymbolics(symbolics);
      if (!symbolics.empty()) {
        text << " Valid values: ";
        for (std::size_t i = 0; i < symbolics.size(); ++i) {
          text << (i ? ", " : "") << symbolics[i].c_str();
        }
        text << ".";
      }
    }
    descriptor.description = text.str();
    return descriptor;
  }

  std::size_t RegisterWritableControlParameters(ControlMapKind kind, bool log_summary)
  {
    GenApi::INodeMap * map = MapOf(kind);
    if (map == nullptr) {
      return 0U;
    }
    GenApi::NodeList_t nodes;
    map->GetNodes(nodes);
    std::size_t registered = 0U;
    for (GenApi::INode * node : nodes) {
      try {
        if (!Writable(node) || !GenApi::IsReadable(node) || node->GetVisibility() == GenApi::Invisible) {
          continue;
        }
        const std::string name = node->GetName().c_str();
        const auto value_kind = ValueKindOf(node);
        if (name.empty() || !value_kind || IsManagedControlNode(kind, name)) {
          continue;
        }
        const std::string parameter = std::string(ControlPrefix(kind)) + "." + name;
        if (bindings_.count(parameter) > 0U) {
          continue;
        }
        const auto descriptor = DescriptorFor(node, *value_kind);
        const ControlBinding binding{kind, *value_kind, name};
        switch (*value_kind) {
          case ControlValueKind::Boolean: {
              const bool current = GenApi::CBooleanPtr(node)->GetValue();
              const bool wanted = declare_parameter<bool>(parameter, current, descriptor);
              if (wanted != current) {pending_.emplace_back(parameter, wanted);}
              break;
            }
          case ControlValueKind::Integer: {
              const std::int64_t current = GenApi::CIntegerPtr(node)->GetValue();
              const std::int64_t wanted = declare_parameter<std::int64_t>(parameter, current, descriptor);
              if (wanted != current) {pending_.emplace_back(parameter, wanted);}
              break;
            }
          case ControlValueKind::Float: {
              const double current = GenApi::CFloatPtr(node)->GetValue();
              const double wanted = declare_parameter<double>(parameter, current, descriptor);
              if (std::abs(wanted - current) > 1e-12) {pending_.emplace_back(parameter, wanted);}
              break;
            }
          case ControlValueKind::Enumeration:
          case ControlValueKind::String: {
              const std::string current = GenApi::CValuePtr(node)->ToString().c_str();
              const std::string wanted = declare_parameter<std::string>(parameter, current, descriptor);
              if (wanted != current) {pending_.emplace_back(parameter, wanted);}
              break;
            }
        }
        bindings_.emplace(parameter, binding);
        ++registered;
      } catch (...) {
        const std::string exception_text = CurrentExceptionText();
        RCLCPP_DEBUG(get_logger(), "Skipping a control parameter: %s", exception_text.c_str());
      }
    }
    if (log_summary) {
      RCLCPP_INFO(get_logger(), "Registered %zu writable %s control parameters.", registered, ControlPrefix(kind));
    }
    return registered;
  }

  void ApplyControlValue(const ControlBinding & binding, const rclcpp::Parameter & parameter)
  {
    GenApi::INode * node = FindNode(MapOf(binding.map_kind), binding.node_name.c_str());
    if (!Writable(node)) {
      throw std::runtime_error("node is not writable in the current camera state");
    }
    switch (binding.value_kind) {
      case ControlValueKind::Boolean:
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          throw std::runtime_error("expected a bool value");
        }
        GenApi::CBooleanPtr(node)->SetValue(parameter.as_bool());
        return;
      case ControlValueKind::Integer:
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
          throw std::runtime_error("expected an integer value");
        }
        GenApi::CIntegerPtr(node)->SetValue(parameter.as_int());
        return;
      case ControlValueKind::Float: {
          double value = 0.0;
          if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            value = parameter.as_double();
          } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            value = static_cast<double>(parameter.as_int());
          } else {
            throw std::runtime_error("expected a floating-point value");
          }
          GenApi::CFloatPtr(node)->SetValue(value);
          return;
        }
      case ControlValueKind::Enumeration: {
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
            throw std::runtime_error("expected a string enum value");
          }
          GenApi::CEnumerationPtr enumeration(node);
          GenApi::CEnumEntryPtr entry = enumeration->GetEntryByName(parameter.as_string().c_str());
          if (!entry.IsValid() || !GenApi::IsAvailable(entry)) {
            throw std::runtime_error("unknown enum symbolic '" + parameter.as_string() + "'");
          }
          enumeration->SetIntValue(entry->GetValue());
          return;
        }
      case ControlValueKind::String:
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          throw std::runtime_error("expected a string value");
        }
        GenApi::CStringPtr(node)->SetValue(parameter.as_string().c_str());
        return;
    }
  }

  // Live set of a camera.* / stream.* / tl_device.* parameter. Nodes that are locked while streaming
  // (found right after StartStream) are written with the stream stopped; ExposureTime and
  // AcquisitionFrameRate move together when exposure.auto_frame_rate is on.
  void ApplyLiveControl(const ControlBinding & binding, const rclcpp::Parameter & parameter)
  {
    if (!stream_started_) {
      ApplyControlValue(binding, parameter);
      return;
    }
    const auto as_double = [&]() {
        return parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
               static_cast<double>(parameter.as_int()) : parameter.as_double();
      };
    if (auto_frame_rate_ && binding.map_kind == ControlMapKind::Camera &&
      (binding.node_name == "ExposureTime" || binding.node_name == "AcquisitionFrameRate"))
    {
      if (binding.node_name == "ExposureTime") {
        const double exposure_us = as_double();
        GenApi::INode * exposure_node = FindNode(camera_map_, "ExposureTime");
        const double fits = Readable(exposure_node) ? GenApi::CFloatPtr(exposure_node)->GetMax() : 0.0;
        const double rate_now = ReadNumber(camera_map_, {"AcquisitionFrameRate"}).value_or(0.0);
        rclcpp::Parameter cap_parameter;
        double cap = std::numeric_limits<double>::infinity();
        if (get_parameter("camera.AcquisitionFrameRate", cap_parameter) &&
          cap_parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
          cap = cap_parameter.as_double();
        }
        const double wanted_rate = std::min(cap, 1.0e6 / (exposure_us + frame_rate_margin_us_));
        if (exposure_us <= fits && rate_now >= 0.8 * wanted_rate) {
          ApplyControlValue(binding, parameter);   // fits the frame period already: no restart
          return;
        }
        WithStreamStopped([&]() {ApplyExposureWithFrameRate(exposure_us);});
        return;
      }
      const double cap = as_double();
      const double exposure_us = ReadNumber(camera_map_, {"ExposureTime"}).value_or(10000.0);
      WithStreamStopped([&]() {ApplyExposureWithFrameRate(exposure_us, cap);});
      return;
    }
    if (restart_for_locked_ && locked_while_streaming_.count(binding.node_name) > 0U) {
      WithStreamStopped([&]() {ApplyControlValue(binding, parameter);});
      RCLCPP_INFO(
        get_logger(), "Restarted the stream to apply %s (locked while streaming).", parameter.get_name().c_str());
      return;
    }
    ApplyControlValue(binding, parameter);
  }

  // Bindings that were writable before StartStream and are not while streaming.
  void FindNodesLockedWhileStreaming()
  {
    locked_while_streaming_.clear();
    for (const auto & [name, binding] : bindings_) {
      (void)name;
      try {
        if (!Writable(FindNode(MapOf(binding.map_kind), binding.node_name.c_str()))) {
          locked_while_streaming_.insert(binding.node_name);
        }
      } catch (...) {
      }
    }
    std::string list;
    for (const auto & node : locked_while_streaming_) {
      list += (list.empty() ? "" : ", ") + node;
    }
    RCLCPP_INFO(
      get_logger(), "%zu control node(s) are locked while streaming%s%s", locked_while_streaming_.size(),
      restart_for_locked_ ? "; setting one restarts the stream: " : ": ", list.c_str());
  }

  void ApplyPendingOverrides()
  {
    auto pending = std::move(pending_);
    pending_.clear();
    std::string last_error;
    for (std::size_t pass = 0; pass < std::max<std::size_t>(1U, pending.size()) && !pending.empty(); ++pass) {
      std::stable_sort(
        pending.begin(), pending.end(), [this](const rclcpp::Parameter & a, const rclcpp::Parameter & b) {
          const int pa = Priority(bindings_.at(a.get_name()).value_kind);
          const int pb = Priority(bindings_.at(b.get_name()).value_kind);
          return pa != pb ? pa < pb : a.get_name() < b.get_name();
        });
      std::vector<rclcpp::Parameter> next;
      std::size_t applied = 0U;
      for (const auto & parameter : pending) {
        try {
          ApplyControlValue(bindings_.at(parameter.get_name()), parameter);
          ++applied;
        } catch (...) {
          const std::string exception_text = CurrentExceptionText();
          last_error = "Failed to apply startup override '" + parameter.get_name() + "': " + exception_text.c_str();
          next.push_back(parameter);
        }
      }
      if (next.empty()) {
        RCLCPP_INFO(get_logger(), "Applied %zu startup camera control overrides.", pending.size());
        return;
      }
      if (applied == 0U) {
        throw std::runtime_error(last_error);
      }
      pending = std::move(next);
    }
    if (!pending.empty()) {
      throw std::runtime_error(last_error);
    }
  }

  // With exposure.auto_frame_rate the startup ExposureTime is held back and applied last, together
  // with the frame rate it needs (a 2 s exposure cannot be written while the camera runs at 3 Hz).
  void HoldBackExposureOverride()
  {
    if (!auto_frame_rate_) {
      return;
    }
    pending_.erase(
      std::remove_if(pending_.begin(), pending_.end(), [](const rclcpp::Parameter & parameter) {
        return parameter.get_name() == "camera.ExposureTime";
      }),
      pending_.end());
  }

  void InitializeControlParameters()
  {
    RegisterWritableControlParameters(ControlMapKind::Camera, true);
    RegisterWritableControlParameters(ControlMapKind::Stream, true);
    RegisterWritableControlParameters(ControlMapKind::TlDevice, true);
    HoldBackExposureOverride();
    ApplyPendingOverrides();
    // Overrides can unlock nodes that were not writable when the camera was opened (ExposureTime
    // after ExposureAuto=Off, DeviceLinkThroughputLimit after its Mode=On, …).
    for (int round = 0; round < 4; ++round) {
      const std::size_t added =
        RegisterWritableControlParameters(ControlMapKind::Camera, false) +
        RegisterWritableControlParameters(ControlMapKind::Stream, false) +
        RegisterWritableControlParameters(ControlMapKind::TlDevice, false);
      if (added == 0U) {
        break;
      }
      RCLCPP_INFO(get_logger(), "Registered %zu control parameters unlocked by the startup overrides.", added);
      HoldBackExposureOverride();
      ApplyPendingOverrides();
    }
    if (auto_frame_rate_) {
      const auto overrides = get_node_parameters_interface()->get_parameter_overrides();
      const auto exposure = overrides.find("camera.ExposureTime");
      if (exposure != overrides.end()) {
        const rclcpp::ParameterValue & value = exposure->second;
        const double exposure_us = value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
          static_cast<double>(value.get<std::int64_t>()) : value.get<double>();
        ApplyExposureWithFrameRate(exposure_us);
      }
    }
    // Whatever is still unregistered never reached the camera.
    std::string ignored;
    std::size_t count = 0U;
    for (const auto & [name, value] : get_node_parameters_interface()->get_parameter_overrides()) {
      (void)value;
      for (const auto kind : {ControlMapKind::Camera, ControlMapKind::Stream, ControlMapKind::TlDevice}) {
        const std::string prefix = std::string(ControlPrefix(kind)) + ".";
        if (name.rfind(prefix, 0) == 0 && bindings_.count(name) == 0U &&
          !IsManagedControlNode(kind, name.substr(prefix.size())))
        {
          ignored += (count++ == 0U ? "" : ", ") + name;
        }
      }
    }
    if (count > 0U) {
      RCLCPP_ERROR(
        get_logger(),
        "%zu startup camera override(s) NOT applied — the GenICam node is locked, unavailable or absent "
        "in this camera state: %s", count, ignored.c_str());
    }
    control_callback_ = add_on_set_parameters_callback(
      std::bind(&LucidRawCameraNode::OnSetParameters, this, std::placeholders::_1));
  }

  std::optional<std::string> Mismatch(const ControlBinding & binding, const rclcpp::Parameter & wanted)
  {
    try {
      GenApi::INode * node = FindNode(MapOf(binding.map_kind), binding.node_name.c_str());
      if (!Readable(node)) {
        return std::nullopt;
      }
      switch (binding.value_kind) {
        case ControlValueKind::Boolean: {
            const bool current = GenApi::CBooleanPtr(node)->GetValue();
            return current == wanted.as_bool() ? std::nullopt :
                   std::optional<std::string>(current ? "true" : "false");
          }
        case ControlValueKind::Integer: {
            GenApi::CIntegerPtr integer(node);
            const std::int64_t current = integer->GetValue();
            std::int64_t increment = 1;
            try {
              increment = std::max<std::int64_t>(1, integer->GetInc());
            } catch (const GenICam::GenericException &) {
            }
            const double tolerance = std::max<double>(
              static_cast<double>(increment), 0.005 * std::abs(static_cast<double>(wanted.as_int())));
            return std::abs(static_cast<double>(current - wanted.as_int())) <= tolerance ?
                   std::nullopt : std::optional<std::string>(std::to_string(current));
          }
        case ControlValueKind::Float: {
            if (wanted.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
              return std::nullopt;
            }
            const double current = GenApi::CFloatPtr(node)->GetValue();
            if (std::abs(current - wanted.as_double()) <= std::max(1e-6, 0.005 * std::abs(wanted.as_double()))) {
              return std::nullopt;
            }
            std::ostringstream text;
            text << current;
            return text.str();
          }
        case ControlValueKind::Enumeration:
        case ControlValueKind::String: {
            const std::string current = GenApi::CValuePtr(node)->ToString().c_str();
            return current == wanted.as_string() ? std::nullopt : std::optional<std::string>(current);
          }
      }
    } catch (...) {
    }
    return std::nullopt;
  }

  // Read every configured override back once the startup sequence is done: paired registers
  // recompute each other (DeviceLinkThroughputLimit vs GevSCPD on the FLIR rig, 2026-09-22).
  void VerifyControlOverridesTookEffect()
  {
    auto find_drift = [this]() {
        std::vector<std::pair<rclcpp::Parameter, std::string>> drift;
        for (const auto & entry : get_node_parameters_interface()->get_parameter_overrides()) {
          const auto binding = bindings_.find(entry.first);
          if (binding == bindings_.end() ||
            (auto_frame_rate_ && binding->second.node_name == "AcquisitionFrameRate"))
          {
            continue;   // with exposure.auto_frame_rate the parameter is a cap, the camera may run slower
          }
          rclcpp::Parameter wanted;
          if (!get_parameter(entry.first, wanted)) {
            continue;
          }
          if (const auto current = Mismatch(binding->second, wanted)) {
            drift.emplace_back(wanted, *current);
          }
        }
        std::stable_sort(drift.begin(), drift.end(), [this](const auto & a, const auto & b) {
            const auto & ba = bindings_.at(a.first.get_name());
            const auto & bb = bindings_.at(b.first.get_name());
            const int ra = Priority(ba.value_kind) * 2 + (ba.node_name == "DeviceLinkThroughputLimit" ? 1 : 0);
            const int rb = Priority(bb.value_kind) * 2 + (bb.node_name == "DeviceLinkThroughputLimit" ? 1 : 0);
            return ra < rb;
          });
        return drift;
      };
    const auto drift = find_drift();
    if (drift.empty()) {
      return;
    }
    std::string fixed;
    for (const auto & [wanted, current] : drift) {
      try {
        ApplyControlValue(bindings_.at(wanted.get_name()), wanted);
      } catch (...) {
      }
      fixed += (fixed.empty() ? "" : ", ") + wanted.get_name() + " (" + current + " -> " + wanted.value_to_string() + ")";
    }
    const auto still = find_drift();
    if (still.size() < drift.size()) {
      RCLCPP_WARN(
        get_logger(), "Re-applied %zu startup camera setting(s) that another setting had overwritten: %s",
        drift.size() - still.size(), fixed.c_str());
    }
    if (!still.empty()) {
      std::string text;
      for (const auto & [wanted, current] : still) {
        text += (text.empty() ? "" : ", ") + wanted.get_name() + "=" + wanted.value_to_string() +
          " but camera has " + current;
      }
      RCLCPP_ERROR(
        get_logger(),
        "%zu startup camera setting(s) not in effect after startup — the camera changed or rejected them: %s",
        still.size(), text.c_str());
    }
  }

  // ======================================================================================
  // raw.* : selector-bound nodes
  // ======================================================================================

  // Writes value_name with selector = entry and returns the value read back.
  double WriteSelected(
    const char * selector_name, const char * entry_name, const char * value_name, double value, const char * hint)
  {
    GenApi::INode * selector = FindNode(camera_map_, selector_name);
    if (Readable(selector)) {
      if (!SetEnumeration(camera_map_, selector_name, entry_name) &&
        ReadText(camera_map_, {selector_name}).value_or("") != entry_name)
      {
        throw std::runtime_error(std::string(selector_name) + " cannot select '" + entry_name + "'");
      }
    }
    GenApi::INode * node = FindNode(camera_map_, value_name);
    if (!Writable(node)) {
      throw std::runtime_error(std::string(value_name) + " is not writable in the current camera state" + hint);
    }
    if (node->GetPrincipalInterfaceType() == GenApi::intfIInteger) {
      GenApi::CIntegerPtr(node)->SetValue(static_cast<std::int64_t>(std::llround(value)));
      return static_cast<double>(GenApi::CIntegerPtr(node)->GetValue());
    }
    GenApi::CFloatPtr(node)->SetValue(value);
    return GenApi::CFloatPtr(node)->GetValue();
  }

  double ReadSelected(const char * selector_name, const char * entry_name, const char * value_name)
  {
    try {
      GenApi::INode * selector = FindNode(camera_map_, selector_name);
      std::optional<std::string> restore;
      if (Readable(selector)) {
        const std::string current = ReadText(camera_map_, {selector_name}).value_or("");
        if (current != entry_name) {
          if (!SetEnumeration(camera_map_, selector_name, entry_name)) {
            return kNaN;
          }
          restore = current;
        }
      }
      const double value = ReadNumber(camera_map_, {value_name}).value_or(kNaN);
      if (restore) {
        SetEnumeration(camera_map_, selector_name, *restore);
      }
      return value;
    } catch (...) {
      return kNaN;
    }
  }

  struct RawSelectorItem
  {
    const char * parameter;
    const char * selector;
    const char * entry;
    const char * value_node;
    const char * hint;
    std::vector<const char *> same_register;
  };

  static const std::vector<RawSelectorItem> & RawSelectorItems()
  {
    static const std::vector<RawSelectorItem> items = {
      {"raw.balance_ratio_red", "BalanceRatioSelector", "Red", "BalanceRatio", " (BalanceWhiteAuto must be Off)",
        {"camera.BalanceRatio"}},
      {"raw.balance_ratio_green", "BalanceRatioSelector", "Green", "BalanceRatio", " (BalanceWhiteAuto must be Off)",
        {"camera.BalanceRatio"}},
      {"raw.balance_ratio_blue", "BalanceRatioSelector", "Blue", "BalanceRatio", " (BalanceWhiteAuto must be Off)",
        {"camera.BalanceRatio"}},
      {"raw.black_level_raw", "BlackLevelSelector", "All", "BlackLevelRaw", "", {"camera.BlackLevelRaw", "camera.BlackLevel"}},
    };
    return items;
  }

  double RawSelectorValue(const std::string & parameter) const
  {
    if (parameter == "raw.balance_ratio_red") {return raw_balance_ratio_red_;}
    if (parameter == "raw.balance_ratio_green") {return raw_balance_ratio_green_;}
    if (parameter == "raw.balance_ratio_blue") {return raw_balance_ratio_blue_;}
    if (parameter == "raw.black_level_raw") {
      return raw_black_level_raw_ < 0 ? kNaN : static_cast<double>(raw_black_level_raw_);
    }
    return kNaN;
  }

  // After the camera.* read-back pass on purpose, so a camera.* key on the same register cannot
  // overwrite it. A mismatch is the ERROR phrase DM_clipGUI turns into an orange row.
  void ApplyRawSelectorParameters(bool startup)
  {
    const auto overrides = get_node_parameters_interface()->get_parameter_overrides();
    std::string applied;
    std::string failed;
    std::size_t failures = 0U;
    for (const auto & item : RawSelectorItems()) {
      const double value = RawSelectorValue(item.parameter);
      if (std::isnan(value)) {
        continue;
      }
      if (startup) {
        for (const char * key : item.same_register) {
          if (overrides.count(key) > 0U) {
            RCLCPP_WARN(get_logger(), "%s is also set; %s wins (applied last). Drop %s from the profile.",
              key, item.parameter, key);
          }
        }
      }
      try {
        const double readback = WriteSelected(item.selector, item.entry, item.value_node, value, item.hint);
        std::ostringstream text;
        text << item.parameter << "=" << readback;
        applied += (applied.empty() ? "" : ", ") + text.str();
        if (std::abs(readback - value) > std::max(1e-6, 0.005 * std::abs(value))) {
          std::ostringstream fail;
          fail << item.parameter << "=" << value << " but camera has " << readback;
          failed += (failed.empty() ? "" : ", ") + fail.str();
          ++failures;
        }
      } catch (...) {
        const std::string exception_text = CurrentExceptionText();
        failed += (failed.empty() ? "" : ", ") + std::string(item.parameter) + ": " + exception_text.c_str();
        ++failures;
      }
    }
    if (!applied.empty()) {
      RCLCPP_INFO(get_logger(), "Applied raw selector settings: %s", applied.c_str());
    }
    if (failures > 0U) {
      RCLCPP_ERROR(
        get_logger(),
        "%zu startup camera setting(s) not in effect after startup — the camera changed or rejected them: %s",
        failures, failed.c_str());
    }
  }

  // ======================================================================================
  // Chunk data, link log
  // ======================================================================================

  void ApplyChunkData()
  {
    chunk_.clear();
    if (!chunk_data_enable_) {
      SetBoolean(camera_map_, "ChunkModeActive", false);
      return;
    }
    try {
      if (!SetBoolean(camera_map_, "ChunkModeActive", true)) {
        RCLCPP_WARN(get_logger(), "ChunkModeActive is not writable; frame values come from node reads.");
        return;
      }
      std::string summary;
      for (const auto & name : chunk_data_selectors_) {
        bool on = false;
        if (SetEnumeration(camera_map_, "ChunkSelector", name)) {
          on = SetBoolean(camera_map_, "ChunkEnable", true) || ReadBool(camera_map_, {"ChunkEnable"}).value_or(false);
        } else {
          RCLCPP_WARN(get_logger(), "chunk '%s' is not supported by this camera", name.c_str());
        }
        if (on) {
          chunk_.insert(name);
        }
        summary += (summary.empty() ? "" : " ") + name + "=" + (on ? "on" : "n/a");
      }
      RCLCPP_INFO(get_logger(), "Chunk data: %s", summary.c_str());
    } catch (...) {
      const std::string exception_text = CurrentExceptionText();
      RCLCPP_WARN(get_logger(), "Could not enable chunk data (%s).", exception_text.c_str());
    }
  }

  bool ChunkOn(const char * name) const
  {
    return chunk_.count(name) > 0U;
  }

  void LogLinkSettings()
  {
    RCLCPP_INFO(
      get_logger(),
      "Link: throughput limit mode %s, limit %.1f MB/s, packet %.0f B (auto-negotiate %s), %s %s, ADC %s.",
      ReadText(camera_map_, {"DeviceLinkThroughputLimitMode"}).value_or("?").c_str(),
      ReadNumber(camera_map_, {"DeviceLinkThroughputLimit"}).value_or(kNaN) / 1e6,
      ReadNumber(camera_map_, {"DeviceStreamChannelPacketSize", "GevSCPSPacketSize"}).value_or(kNaN),
      ReadBool(stream_map_, {"StreamAutoNegotiatePacketSize"}).value_or(false) ? "on" : "off",
      ReadText(camera_map_, {"PixelFormat"}).value_or("?").c_str(),
      ReadText(camera_map_, {"HDROutput"}).value_or("").c_str(),
      ReadText(camera_map_, {"ADCBitDepth", "AdcBitDepth"}).value_or("?").c_str());
  }

  // ======================================================================================
  // Session metadata, housekeeping, live parameter changes
  // ======================================================================================

  struct SessionInfo
  {
    std::string serial, model, firmware_version, adc_bit_depth;
    bool acquisition_frame_rate_enable{false};
    double acquisition_frame_rate_hz{kNaN};
    std::string exposure_auto, gain_auto, balance_white_auto;
    double balance_ratio_red{kNaN}, balance_ratio_green{kNaN}, balance_ratio_blue{kNaN};
    bool lut_enable{false};
    std::string cfa_pattern;
    std::int64_t black_level_raw{-1};
    std::string hdr_output;
    bool hdr_image_enhancement_enable{false}, hdr_digital_clamping_enable{false}, hdr_tuning_enable{false};
    std::array<double, 4> hdr_channel_analog_gain_db{kNaN, kNaN, kNaN, kNaN};
    std::array<double, 4> hdr_channel_exposure_time_us{kNaN, kNaN, kNaN, kNaN};
    bool balance_white_enable{false}, color_transformation_enable{false};
    std::string lut_tone_mapping;
    bool reverse_x{false}, reverse_y{false};
    std::string sensor_shutter_mode;
  };

  void RefreshSessionInfo()
  {
    if (camera_map_ == nullptr) {
      return;
    }
    SessionInfo s;
    s.serial = ReadText(tl_device_map_, {"DeviceSerialNumber"}).value_or(
      ReadText(camera_map_, {"DeviceSerialNumber"}).value_or(""));
    s.model = ReadText(tl_device_map_, {"DeviceModelName"}).value_or(
      ReadText(camera_map_, {"DeviceModelName"}).value_or(""));
    s.firmware_version = ReadText(camera_map_, {"DeviceFirmwareVersion"}).value_or("");
    s.adc_bit_depth = ReadText(camera_map_, {"ADCBitDepth", "AdcBitDepth"}).value_or("");
    s.acquisition_frame_rate_enable = ReadBool(camera_map_, {"AcquisitionFrameRateEnable"}).value_or(false);
    s.acquisition_frame_rate_hz = ReadNumber(camera_map_, {"AcquisitionFrameRate"}).value_or(kNaN);
    s.exposure_auto = ReadText(camera_map_, {"ExposureAuto"}).value_or("");
    s.gain_auto = ReadText(camera_map_, {"GainAuto"}).value_or("");
    s.balance_white_auto = ReadText(camera_map_, {"BalanceWhiteAuto"}).value_or("");
    s.balance_white_enable = ReadBool(camera_map_, {"BalanceWhiteEnable"}).value_or(false);
    s.balance_ratio_red = ReadSelected("BalanceRatioSelector", "Red", "BalanceRatio");
    s.balance_ratio_green = ReadSelected("BalanceRatioSelector", "Green", "BalanceRatio");
    s.balance_ratio_blue = ReadSelected("BalanceRatioSelector", "Blue", "BalanceRatio");
    s.lut_enable = ReadBool(camera_map_, {"LUTEnable"}).value_or(false);
    s.lut_tone_mapping = ReadText(camera_map_, {"LUTToneMapping"}).value_or("");
    s.color_transformation_enable = ReadBool(camera_map_, {"ColorTransformationEnable"}).value_or(false);
    const auto format = ParseRawFormat(ReadText(camera_map_, {"PixelFormat"}).value_or(pixel_format_));
    s.cfa_pattern = format ? format->cfa : "";
    s.black_level_raw = static_cast<std::int64_t>(
      std::llround(ReadSelected("BlackLevelSelector", "All", "BlackLevelRaw")));
    s.hdr_output = ReadText(camera_map_, {"HDROutput"}).value_or("");
    s.hdr_image_enhancement_enable = ReadBool(camera_map_, {"HDRImageEnhancementEnable"}).value_or(false);
    s.hdr_digital_clamping_enable = ReadBool(camera_map_, {"HDRDigitalClampingEnable"}).value_or(false);
    s.hdr_tuning_enable = ReadBool(camera_map_, {"HDRTuningEnable"}).value_or(false);
    // Per-channel analog gain / exposure behind the integer selector HDRTuningChannelSelector.
    if (const auto original = ReadInteger(camera_map_, "HDRTuningChannelSelector")) {
      GenApi::INode * selector = FindNode(camera_map_, "HDRTuningChannelSelector");
      for (int channel = 0; channel < 4; ++channel) {
        try {
          if (Writable(selector)) {
            GenApi::CIntegerPtr(selector)->SetValue(channel);
          } else if (channel != *original) {
            continue;
          }
          s.hdr_channel_analog_gain_db[channel] = ReadNumber(camera_map_, {"HDRChannelAnalogGain"}).value_or(kNaN);
          s.hdr_channel_exposure_time_us[channel] = ReadNumber(camera_map_, {"HDRChannelExposureTime"}).value_or(kNaN);
        } catch (...) {
        }
      }
      try {
        if (Writable(selector)) {
          GenApi::CIntegerPtr(selector)->SetValue(*original);
        }
      } catch (...) {
      }
    }
    s.reverse_x = ReadBool(camera_map_, {"ReverseX"}).value_or(false);
    s.reverse_y = ReadBool(camera_map_, {"ReverseY"}).value_or(false);
    s.sensor_shutter_mode = ReadText(camera_map_, {"SensorShutterMode"}).value_or("");
    device_temperature_c_.store(ReadNumber(camera_map_, {"DeviceTemperature"}).value_or(kNaN));
    device_uptime_s_.store(ReadNumber(camera_map_, {"DeviceUpTime", "DeviceUptime"}).value_or(kNaN));
    raw_white_level_dn_.store(EffectiveWhiteLevel(s, format));
    std::lock_guard<std::mutex> lock(session_mutex_);
    session_ = s;
  }

  // White level of image_raw. raw.white_level_dn > 0 wins; -1 (auto) uses the clip points measured on
  // the TRI054S (2026-10-19, HDRChannel0, BlackLevelRaw 256): in BayerRG24 a channel output is the
  // 12-bit code plus the pedestal (max 4095 + 256 = 4351); BayerRG16 / 12p carry min(that, 4095), so
  // they lose the top 256 codes (max 65520 = 4095 << 4, or 4095 << raw12p_shift). The HDR composite's
  // clip point is not measured: 0 (unknown).
  std::uint32_t EffectiveWhiteLevel(const SessionInfo & s, const std::optional<RawFormat> & format) const
  {
    if (raw_white_level_param_ >= 0) {
      return static_cast<std::uint32_t>(std::min<std::int64_t>(raw_white_level_param_, 0xFFFFFFFFLL));
    }
    if (!format) {
      return 0U;
    }
    if (format->bits == 8) {
      return 255U;
    }
    if (s.hdr_output.rfind("HDRChannel", 0) != 0) {
      return 0U;
    }
    switch (format->bits) {
      case 24: return 4095U + static_cast<std::uint32_t>(std::max<std::int64_t>(0, s.black_level_raw));
      case 16: return 65520U;
      case 12: return 4095U << raw12p_shift_;
      default: return 0U;
    }
  }

  void OnCameraSettingsChanged()
  {
    RefreshSessionInfo();
    if (!nodemap_dump_dir_.empty() && stream_started_) {
      nodemap_dump_due_ = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    }
  }

  void Housekeeping()
  {
    if (camera_map_ == nullptr || !running_.load()) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (temperature_interval_sec_ > 0.0 && now >= next_temperature_read_) {
      next_temperature_read_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(temperature_interval_sec_));
      device_temperature_c_.store(ReadNumber(camera_map_, {"DeviceTemperature"}).value_or(kNaN));
      device_uptime_s_.store(ReadNumber(camera_map_, {"DeviceUpTime", "DeviceUptime"}).value_or(kNaN));
      std::string wb_auto;
      {
        std::lock_guard<std::mutex> lock(session_mutex_);
        wb_auto = session_.balance_white_auto;
      }
      if (!wb_auto.empty() && NormalizeName(wb_auto) != "off") {
        const double red = ReadSelected("BalanceRatioSelector", "Red", "BalanceRatio");
        const double green = ReadSelected("BalanceRatioSelector", "Green", "BalanceRatio");
        const double blue = ReadSelected("BalanceRatioSelector", "Blue", "BalanceRatio");
        std::lock_guard<std::mutex> lock(session_mutex_);
        session_.balance_ratio_red = red;
        session_.balance_ratio_green = green;
        session_.balance_ratio_blue = blue;
      }
    }
    if (nodemap_dump_due_ && now >= *nodemap_dump_due_) {
      nodemap_dump_due_.reset();
      DumpNodeMap("parameter change");
    }
  }

  rcl_interfaces::msg::SetParametersResult OnSetParameters(const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool camera_changed = false;
    const auto finish = [&]() {
        if (camera_changed) {
          OnCameraSettingsChanged();
        }
        return result;
      };
    for (const auto & parameter : parameters) {
      const std::string & name = parameter.get_name();
      const auto fail = [&](const std::string & reason) {
          result.successful = false;
          result.reason = "Failed to set '" + name + "': " + reason;
        };
      const auto as_double = [&]() -> std::optional<double> {
          if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            return parameter.as_double();
          }
          if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            return static_cast<double>(parameter.as_int());
          }
          return std::nullopt;
        };

      if (const auto binding = bindings_.find(name); binding != bindings_.end()) {
        try {
          ApplyLiveControl(binding->second, parameter);
          camera_changed = true;
        } catch (...) {
          const std::string exception_text = CurrentExceptionText();
          fail(exception_text.c_str());
          return finish();
        }
        continue;
      }

      bool raw_item = false;
      for (const auto & item : RawSelectorItems()) {
        if (name != item.parameter) {
          continue;
        }
        raw_item = true;
        const auto value = as_double();
        if (!value) {
          fail("expected a number");
          return finish();
        }
        const bool leave = std::isnan(*value) || (name == "raw.black_level_raw" && *value < 0.0);
        if (!leave && camera_map_ != nullptr) {
          try {
            WriteSelected(item.selector, item.entry, item.value_node, *value, item.hint);
            camera_changed = true;
          } catch (...) {
            const std::string exception_text = CurrentExceptionText();
            fail(exception_text.c_str());
            return finish();
          }
        }
        if (name == "raw.balance_ratio_red") {
          raw_balance_ratio_red_ = *value;
        } else if (name == "raw.balance_ratio_green") {
          raw_balance_ratio_green_ = *value;
        } else if (name == "raw.balance_ratio_blue") {
          raw_balance_ratio_blue_ = *value;
        } else {
          raw_black_level_raw_ = static_cast<std::int64_t>(std::llround(*value));
        }
      }
      if (raw_item) {
        continue;
      }

      if (name == "pixel_format" || name == "raw24_container" || name == "raw12p_shift") {
        try {
          if (name == "pixel_format") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
              !ParseRawFormat(parameter.as_string()))
            {
              fail("expected a Bayer/Mono 8, 12p, 16 or 24 pixel format");
              return finish();
            }
            const std::string format = parameter.as_string();
            WithStreamStopped([&]() {
                if (!SetEnumeration(camera_map_, "PixelFormat", format)) {
                  throw std::runtime_error("the camera refused PixelFormat " + format);
                }
                pixel_format_ = format;
              });
            RCLCPP_INFO(get_logger(), "PixelFormat set to '%s' (stream restarted).", format.c_str());
            camera_changed = true;
          } else if (name == "raw24_container") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
              fail("expected 'uint32' or 'packed'");
              return finish();
            }
            const std::string container = NormalizeName(parameter.as_string()) == "packed" ? "packed" : "uint32";
            WithStreamStopped([&]() {raw24_container_ = container;}, false);
          } else {
            const auto value = as_double();
            if (!value) {
              fail("expected an integer 0..4");
              return finish();
            }
            const int shift = std::clamp(static_cast<int>(*value), 0, 4);
            WithStreamStopped([&]() {raw12p_shift_ = shift;}, false);
          }
        } catch (...) {
          const std::string exception_text = CurrentExceptionText();
          fail(exception_text);
          return finish();
        }
        continue;
      }
      if (name == "exposure.auto_frame_rate" || name == "live.restart_stream_for_locked") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          fail("expected a bool");
          return finish();
        }
        (name == "exposure.auto_frame_rate" ? auto_frame_rate_ : restart_for_locked_) = parameter.as_bool();
        continue;
      }
      if (name == "trigger.mode" || name.rfind("trigger.action.", 0) == 0) {
        try {
          if (name == "trigger.mode") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
              (NormalizeName(parameter.as_string()) != "off" && NormalizeName(parameter.as_string()) != "action"))
            {
              fail("expected 'off' or 'action'");
              return finish();
            }
            trigger_mode_ = NormalizeName(parameter.as_string());
          } else if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER || parameter.as_int() < 0) {
            fail("expected a non-negative integer");
            return finish();
          } else {
            (name == "trigger.action.device_key" ? trigger_device_key_ :
            name == "trigger.action.group_key" ? trigger_group_key_ : trigger_group_mask_) = parameter.as_int();
          }
          if (camera_map_ != nullptr) {
            WithStreamStopped([&]() {ApplyTriggerMode();});
            camera_changed = true;
          }
        } catch (...) {
          const std::string exception_text = CurrentExceptionText();
          fail(exception_text);
          return finish();
        }
        continue;
      }
      if (name == "exposure.frame_rate_margin_us") {
        const auto value = as_double();
        if (!value || *value < 0.0) {
          fail("expected a non-negative number");
          return finish();
        }
        frame_rate_margin_us_ = *value;
        continue;
      }

      if (name == "raw.white_level_dn") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER || parameter.as_int() < -1) {
          fail("expected -1 (auto), 0 (unknown) or a code value");
          return finish();
        }
        raw_white_level_param_ = parameter.as_int();
        camera_changed = true;
      } else if (name == "lens.model" || name == "profile_path" || name == "nodemap_dump_dir") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          fail("expected a string");
          return finish();
        }
        (name == "lens.model" ? lens_model_ : name == "profile_path" ? profile_path_ : nodemap_dump_dir_) =
          parameter.as_string();
        camera_changed = true;
      } else if (name == "lens.f_number" || name == "temperature.interval_sec") {
        const auto value = as_double();
        if (!value) {
          fail("expected a number");
          return finish();
        }
        (name == "lens.f_number" ? lens_f_number_ : temperature_interval_sec_) = *value;
        camera_changed = camera_changed || name == "lens.f_number";
      } else if (name.rfind("preview.", 0) == 0) {
        std::lock_guard<std::mutex> lock(preview_mutex_);
        raw_preview::Settings & s = preview_settings_;
        const auto value = as_double();
        if (name == "preview.white_balance") {
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
            fail("expected 'gray_world' or 'none'");
            return finish();
          }
          s.gray_world = NormalizeName(parameter.as_string()) != "none";
        } else if (!value) {
          fail("expected a number");
          return finish();
        } else if (name == "preview.max_rate_hz") {
          s.max_rate_hz = *value;
        } else if (name == "preview.downscale") {
          s.downscale = std::max(1, static_cast<int>(*value));
        } else if (name == "preview.gain") {
          s.gain = *value;
        } else if (name == "preview.target_mean") {
          s.target_mean = *value;
        } else if (name == "preview.max_gain") {
          s.max_gain = *value;
        } else if (name == "preview.noise_ceiling") {
          s.noise_ceiling = *value;
        } else if (name == "preview.gamma") {
          s.gamma = *value;
        } else if (name == "preview.black_level_dn") {
          s.black_level_dn = *value;
        } else if (name == "preview.jpeg_quality") {
          s.jpeg_quality = std::clamp(static_cast<int>(*value), 1, 100);
        }
      }
    }
    return finish();
  }

  // ======================================================================================
  // Nodemap dump (docs/raw_capture_noise_calibration.md §4.4 in FLIR_control)
  // ======================================================================================

  using BufferPortMemo = std::unordered_map<GenApi::INode *, bool>;

  static bool IsDumpedValueNode(GenApi::INode * node, BufferPortMemo & memo)
  {
    switch (node->GetPrincipalInterfaceType()) {
      case GenApi::intfIInteger:
      case GenApi::intfIFloat:
      case GenApi::intfIBoolean:
      case GenApi::intfIEnumeration:
      case GenApi::intfIString:
        break;
      default:
        return false;
    }
    const std::string name = node->GetName().c_str();
    if (name.rfind("Chunk", 0) == 0 && name != "ChunkModeActive" && name != "ChunkSelector" &&
      name != "ChunkEnable")
    {
      return false;
    }
    return !ReadsBufferPort(node, memo);
  }

  // Values that live in the last frame's chunk data or the last event's data (a port with a ChunkID /
  // EventID), not in camera registers — the TRI054S XML has 22 such registers named N13xx (plus the
  // converters above them), so the name test above misses them. With no buffer attached they read memory
  // the stream already released: after the stream restart for trigger.mode=action no frame re-attaches
  // one, and the next dump segfaulted. Depth-first over the reading children, each node decided once.
  static bool ReadsBufferPort(GenApi::INode * node, BufferPortMemo & memo)
  {
    if (node == nullptr) {
      return false;
    }
    if (const auto it = memo.find(node); it != memo.end()) {
      return it->second;
    }
    memo[node] = false;   // cycle guard
    bool reads = false;
    GenICam::gcstring value;
    GenICam::gcstring attribute;
    if (node->GetPrincipalInterfaceType() == GenApi::intfIPort) {
      reads = node->GetProperty("ChunkID", value, attribute) || node->GetProperty("EventID", value, attribute);
    } else {
      if (node->GetProperty("pPort", value, attribute) && !value.empty()) {
        reads = ReadsBufferPort(node->GetNodeMap()->GetNode(value), memo);
      }
      if (!reads) {
        GenApi::NodeList_t children;
        node->GetChildren(children, GenApi::ctReadingChildren);
        for (GenApi::INode * child : children) {
          if (ReadsBufferPort(child, memo)) {
            reads = true;
            break;
          }
        }
      }
    }
    memo[node] = reads;
    return reads;
  }

  // memo: buffer_port_memo_ — kept for the node's lifetime (the node maps do not change while the device is
  // open), so only the first dump pays for walking the node graph.
  static void AppendNodeValuesJson(
    std::ostringstream & out, GenApi::INodeMap * map, const char * indent, BufferPortMemo & memo)
  {
    std::map<std::string, std::string> values;
    if (map != nullptr) {
      GenApi::NodeList_t nodes;
      map->GetNodes(nodes);
      for (GenApi::INode * node : nodes) {
        try {
          if (!Readable(node) || !IsDumpedValueNode(node, memo)) {
            continue;
          }
          values[node->GetName().c_str()] = GenApi::CValuePtr(node)->ToString().c_str();
        } catch (...) {
        }
      }
    }
    out << "{";
    bool first = true;
    for (const auto & [name, value] : values) {
      out << (first ? "\n" : ",\n") << indent << "  \"" << JsonEscape(name) << "\": \"" << JsonEscape(value) << "\"";
      first = false;
    }
    out << "\n" << indent << "}";
  }

  // Selector-bound nodes, one object per selector entry. ColorTransformation gains are the camera's
  // built-in CCM, HDRTuningChannelSelector the per-channel analog gain / exposure.
  // Values behind selectors — only the entry that is selected now. Stepping a selector through its
  // entries while streaming (the dump used to, then put it back) moved the TRI054S global black level
  // by -9.7 DN at long exposures until the next stream restart (2026-10-20: 1 s dark 275.9 -> 266.2 DN
  // right after a dump, never without one). Per-channel gains, balance ratios and the HDR channel
  // values are in every frame's metadata anyway.
  void AppendSelectedValuesJson(std::ostringstream & out, const char * indent)
  {
    out << "{";
    bool first_selector = true;
    for (const char * selector_name :
      {"BalanceRatioSelector", "ColorTransformationValueSelector", "BlackLevelSelector", "GainSelector",
        "HDRTuningChannelSelector"})
    {
      try {
        GenApi::INode * selector = FindNode(camera_map_, selector_name);
        if (!Readable(selector)) {
          continue;
        }
        const std::string current = GenApi::CValuePtr(selector)->ToString().c_str();
        GenApi::FeatureList_t features;
        GenApi::CSelectorPtr(selector)->GetSelectedFeatures(features);
        out << (first_selector ? "\n" : ",\n") << indent << "  \"" << selector_name << "\": {\"" << JsonEscape(current)
            << "\": {";
        first_selector = false;
        bool first_value = true;
        for (GenApi::IValue * feature : features) {
          try {
            if (feature == nullptr || !GenApi::IsReadable(feature)) {
              continue;
            }
            out << (first_value ? "" : ", ") << "\"" << JsonEscape(feature->GetNode()->GetName().c_str())
                << "\": \"" << JsonEscape(feature->ToString().c_str()) << "\"";
            first_value = false;
          } catch (...) {
          }
        }
        out << "}}";
      } catch (...) {
      }
    }
    out << "\n" << indent << "}";
  }

  void DumpNodeMap(const std::string & reason)
  {
    if (nodemap_dump_dir_.empty() || camera_map_ == nullptr) {
      return;
    }
    try {
      namespace fs = std::filesystem;
      fs::create_directories(nodemap_dump_dir_);
      const auto now = std::chrono::system_clock::now();
      const std::time_t now_t = std::chrono::system_clock::to_time_t(now);
      std::tm local{};
      localtime_r(&now_t, &local);
      char file_stamp[32];
      char iso_stamp[40];
      std::strftime(file_stamp, sizeof(file_stamp), "%Y%m%d_%H%M%S", &local);
      std::strftime(iso_stamp, sizeof(iso_stamp), "%Y-%m-%dT%H:%M:%S%z", &local);
      std::string serial;
      std::string model;
      {
        std::lock_guard<std::mutex> lock(session_mutex_);
        serial = session_.serial.empty() ? "unknown" : session_.serial;
        model = session_.model;
      }
      std::ostringstream out;
      out << "{\n";
      out << "  \"format\": \"lucid_raw_camera nodemap dump v1\",\n";
      out << "  \"reason\": \"" << JsonEscape(reason) << "\",\n";
      out << "  \"time_local\": \"" << iso_stamp << "\",\n";
      out << "  \"time_unix_ns\": " <<
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count() << ",\n";
      out << "  \"ros_node\": \"" << JsonEscape(get_fully_qualified_name()) << "\",\n";
      out << "  \"driver_git\": \"" << JsonEscape(kLucidDriverGitDescribe) << "\",\n";
      out << "  \"profile_path\": \"" << JsonEscape(profile_path_) << "\",\n";
      out << "  \"serial\": \"" << JsonEscape(serial) << "\",\n";
      out << "  \"model\": \"" << JsonEscape(model) << "\",\n";
      out << "  \"lens\": {\"model\": \"" << JsonEscape(lens_model_) << "\", \"f_number\": " <<
        JsonNumberOrNull(lens_f_number_) << "},\n";
      out << "  \"white_level_dn\": " << raw_white_level_dn_.load() << ",\n";
      out << "  \"image_raw\": {\"pixel_format\": \"" << JsonEscape(pixel_format_) << "\", \"raw24_container\": \"" <<
        raw24_container_ << "\", \"raw12p_shift\": " << raw12p_shift_ << "},\n";
      out << "  \"camera\": ";
      AppendNodeValuesJson(out, camera_map_, "  ", buffer_port_memo_);
      out << ",\n  \"selected\": ";
      AppendSelectedValuesJson(out, "  ");
      out << ",\n  \"tl_device\": ";
      AppendNodeValuesJson(out, tl_device_map_, "  ", buffer_port_memo_);
      out << ",\n  \"stream\": ";
      AppendNodeValuesJson(out, stream_map_, "  ", buffer_port_memo_);
      out << "\n}\n";

      const fs::path dir(nodemap_dump_dir_);
      const fs::path path = dir / (serial + "_" + file_stamp + "_nodemap.json");
      const fs::path latest = dir / (serial + "_latest_nodemap.json");
      const fs::path tmp = dir / ("." + serial + "_nodemap.json.tmp");
      {
        std::ofstream file(tmp);
        file << out.str();
        if (!file) {
          throw std::runtime_error("could not write " + tmp.string());
        }
      }
      fs::copy_file(tmp, latest, fs::copy_options::overwrite_existing);
      fs::rename(tmp, path);
      RCLCPP_INFO(get_logger(), "Nodemap dump (%s): %s", reason.c_str(), path.c_str());
    } catch (...) {
      const std::string exception_text = CurrentExceptionText();
      RCLCPP_WARN(get_logger(), "Nodemap dump failed: %s", exception_text.c_str());
    }
  }

  // ======================================================================================
  // header.stamp = exposure start, mapped from the camera clock (as in flir_spinnaker_camera_node)
  // ======================================================================================

  struct ClockSample
  {
    std::int64_t camera_ns;
    std::int64_t host_ns;
  };

  // TimestampLatch bracketed by two host clock reads pins one camera counter value to a host time
  // within half the GVCP round trip.
  void SampleCameraClock()
  {
    try {
      GenApi::INode * latch = FindNode(camera_map_, "TimestampLatch");
      if (!Writable(latch)) {
        if (!clock_warned_) {
          RCLCPP_WARN(get_logger(), "TimestampLatch is not available; header.stamp falls back to host time.");
          clock_warned_ = true;
        }
        return;
      }
      const std::int64_t before = now().nanoseconds();
      GenApi::CCommandPtr(latch)->Execute();
      const auto value = ReadInteger(camera_map_, "TimestampLatchValue");
      const std::int64_t after = now().nanoseconds();
      if (!value || *value <= 0) {
        return;
      }
      std::lock_guard<std::mutex> lock(clock_mutex_);
      clock_samples_.push_back({*value, before + (after - before) / 2});
      while (clock_samples_.size() > 16U) {
        clock_samples_.pop_front();
      }
      // Least-squares line host = a + b * camera through recent samples (b ~ 1 + drift).
      const auto & first = clock_samples_.front();
      double sx = 0, sy = 0, sxx = 0, sxy = 0;
      const double n = static_cast<double>(clock_samples_.size());
      for (const auto & sample : clock_samples_) {
        const double x = static_cast<double>(sample.camera_ns - first.camera_ns);
        const double y = static_cast<double>(sample.host_ns - first.host_ns);
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
      }
      const double denominator = n * sxx - sx * sx;
      clock_slope_ = (clock_samples_.size() >= 2U && std::abs(denominator) > 1e-9) ?
        std::clamp((n * sxy - sx * sy) / denominator, 0.999, 1.001) : 1.0;
      clock_intercept_ns_ = static_cast<double>(first.host_ns) + (sy - clock_slope_ * sx) / n;
      clock_origin_camera_ns_ = first.camera_ns;
      clock_valid_ = true;
    } catch (...) {
      const std::string exception_text = CurrentExceptionText();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Camera clock latch failed: %s", exception_text.c_str());
    }
  }

  void ClockSyncLoop()
  {
    auto next = std::chrono::steady_clock::now();
    while (running_.load()) {
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(std::max(0.2, timestamp_latch_interval_sec_)));
      while (running_.load() && std::chrono::steady_clock::now() < next) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      if (running_.load()) {
        SampleCameraClock();
      }
    }
  }

  rclcpp::Time ResolveStamp(std::uint64_t camera_ns, double exposure_us, const rclcpp::Time & host_stamp)
  {
    if (timestamp_mode_ != "camera_latched" || camera_ns == 0U) {
      return host_stamp;
    }
    std::lock_guard<std::mutex> lock(clock_mutex_);
    if (!clock_valid_) {
      return host_stamp;
    }
    double start_ns = static_cast<double>(static_cast<std::int64_t>(camera_ns) - clock_origin_camera_ns_);
    if (timestamp_exposure_latch_ == "end" && std::isfinite(exposure_us)) {
      start_ns -= exposure_us * 1000.0;
    }
    const auto host_ns = static_cast<std::int64_t>(std::llround(clock_intercept_ns_ + clock_slope_ * start_ns));
    return rclcpp::Time(host_ns, get_clock()->get_clock_type());
  }

  // ======================================================================================
  // Acquisition
  // ======================================================================================

  struct FrameInfo
  {
    std::uint64_t frame_id{0};
    std::uint64_t timestamp_ns{0};
    double exposure_us{kNaN};
    double gain_db{kNaN};
    double black_level{kNaN};
    std::string pixel_format;
    bool chunk_read{false};
    bool crc_ok{false};
  };

  FrameInfo ReadFrameInfo(Arena::IImage * image)
  {
    FrameInfo info;
    info.frame_id = image->GetFrameId();
    info.timestamp_ns = image->GetTimestampNs();
    if (!chunk_.empty() && image->HasChunkData()) {
      Arena::IChunkData * chunk = image->AsChunkData();
      try {
        const auto float_chunk = [&](const char * name) {
            GenApi::INode * node = chunk->GetChunk(name);
            return Readable(node) ? GenApi::CFloatPtr(node)->GetValue() : kNaN;
          };
        if (ChunkOn("ExposureTime")) {
          info.exposure_us = float_chunk("ChunkExposureTime");
        }
        if (ChunkOn("Gain")) {
          info.gain_db = float_chunk("ChunkGain");
        }
        if (ChunkOn("BlackLevel")) {
          info.black_level = float_chunk("ChunkBlackLevel");
        }
        if (ChunkOn("Timestamp")) {
          GenApi::INode * node = chunk->GetChunk("ChunkTimestamp");
          if (Readable(node)) {
            const std::int64_t value = GenApi::CIntegerPtr(node)->GetValue();
            if (value > 0) {
              info.timestamp_ns = static_cast<std::uint64_t>(value);
            }
          }
        }
        if (ChunkOn("PixelFormat")) {
          GenApi::INode * node = chunk->GetChunk("ChunkPixelFormat");
          if (Readable(node)) {
            info.pixel_format = GenApi::CValuePtr(node)->ToString().c_str();
          }
        }
        info.chunk_read = true;
      } catch (...) {
        info.chunk_read = false;
      }
      if (ChunkOn("CRC")) {
        try {
          info.crc_ok = chunk->VerifyCRC();
        } catch (...) {
          info.crc_ok = false;
        }
      }
    }
    return info;
  }

  // image_raw from the Arena buffer; also hands the preview thread its copy when one is due.
  sensor_msgs::msg::Image BuildRawImage(
    Arena::IImage * image, const RawFormat & format, const rclcpp::Time & stamp, bool want_preview)
  {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.width = static_cast<std::uint32_t>(image->GetWidth());
    msg.height = static_cast<std::uint32_t>(image->GetHeight());
    msg.is_bigendian = IsHostBigEndian() ? 1U : 0U;
    const std::size_t pixels = static_cast<std::size_t>(msg.width) * msg.height;
    const std::uint8_t * src = image->GetData();
    const std::size_t available = image->GetSizeFilled();
    const bool big_endian = image->GetPixelEndianness() == Arena::PixelEndiannessBig;
    const auto need = [&](std::size_t bytes) {
        if (available < bytes) {
          throw std::runtime_error(
            "buffer holds " + std::to_string(available) + " bytes, " + std::to_string(bytes) + " needed");
        }
      };

    if (format.bits == 8) {
      need(pixels);
      msg.encoding = Encoding8(format.cfa);
      msg.step = msg.width;
      msg.data.assign(src, src + pixels);
      if (want_preview) {
        SubmitPreview(msg.data.data(), msg.width, msg.height, format.cfa, 255.0, stamp);
      }
    } else if (format.bits == 12) {
      need((pixels * 3U + 1U) / 2U);
      msg.encoding = Encoding16(format.cfa);
      msg.step = msg.width * 2U;
      msg.data.resize(pixels * 2U);
      auto * dst = reinterpret_cast<std::uint16_t *>(msg.data.data());
      Unpack12p(src, dst, pixels, raw12p_shift_);
      if (want_preview) {
        SubmitPreview(dst, msg.width, msg.height, format.cfa, static_cast<double>(4095U << raw12p_shift_), stamp);
      }
    } else if (format.bits == 16) {
      need(pixels * 2U);
      msg.encoding = Encoding16(format.cfa);
      msg.step = msg.width * 2U;
      msg.data.resize(pixels * 2U);
      if (big_endian != IsHostBigEndian()) {
        for (std::size_t i = 0; i < pixels; ++i) {
          msg.data[2U * i] = src[2U * i + 1U];
          msg.data[2U * i + 1U] = src[2U * i];
        }
      } else {
        std::memcpy(msg.data.data(), src, pixels * 2U);
      }
      if (want_preview) {
        SubmitPreview(reinterpret_cast<const std::uint16_t *>(msg.data.data()), msg.width, msg.height,
          format.cfa, 65535.0, stamp);
      }
    } else {   // 24-bit
      need(pixels * 3U);
      if (raw24_container_ == "packed") {
        msg.encoding = format.cfa.empty() ? "mono24" : "bayer_" + Lower(format.cfa) + "24";
        msg.step = msg.width * 3U;
        msg.data.assign(src, src + pixels * 3U);
        msg.is_bigendian = big_endian ? 1U : 0U;
        if (want_preview) {
          std::vector<std::uint32_t> expanded(pixels);
          Expand24(src, expanded.data(), pixels, big_endian);
          SubmitPreview(expanded.data(), msg.width, msg.height, format.cfa, 16777215.0, stamp);
        }
      } else {
        msg.encoding = sensor_msgs::image_encodings::TYPE_32SC1;
        msg.step = msg.width * 4U;
        msg.data.resize(pixels * 4U);
        auto * dst = reinterpret_cast<std::uint32_t *>(msg.data.data());
        Expand24(src, dst, pixels, big_endian);
        if (want_preview) {
          SubmitPreview(dst, msg.width, msg.height, format.cfa, 16777215.0, stamp);
        }
      }
    }
    return msg;
  }

  static std::string Lower(std::string text)
  {
    for (auto & c : text) {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return text;
  }

  lucid_raw_camera::msg::LucidMetadata BuildMetadata(
    const sensor_msgs::msg::Image & raw, const RawFormat & format, const FrameInfo & info,
    const rclcpp::Time & stamp)
  {
    lucid_raw_camera::msg::LucidMetadata msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.width = raw.width;
    msg.height = raw.height;
    msg.step = raw.step;
    msg.encoding = raw.encoding;
    msg.pixel_format = info.pixel_format.empty() ? format.name : info.pixel_format;
    msg.camera_frame_id = info.frame_id;
    msg.camera_timestamp_ns = info.timestamp_ns;
    const auto pick = [&](const char * chunk_name, double chunk_value, std::initializer_list<const char *> nodes) {
        if (ChunkOn(chunk_name)) {
          return info.chunk_read ? chunk_value : kNaN;
        }
        return ReadNumber(camera_map_, nodes).value_or(kNaN);
      };
    msg.exposure_time_us = pick("ExposureTime", info.exposure_us, {"ExposureTime"});
    msg.gain_db = pick("Gain", info.gain_db, {"Gain"});
    msg.black_level = pick("BlackLevel", info.black_level, {"BlackLevel"});
    msg.chunk_valid = ChunkOn("ExposureTime") && ChunkOn("Gain") && ChunkOn("BlackLevel") && info.chunk_read &&
      std::isfinite(msg.exposure_time_us) && std::isfinite(msg.gain_db) && std::isfinite(msg.black_level);
    msg.crc_ok = ChunkOn("CRC") && info.crc_ok;

    // settled: same exposure / gain / black level as the two previous frames
    const std::array<double, 3> values{msg.exposure_time_us, msg.gain_db, msg.black_level};
    const auto same = [](const std::array<double, 3> & a, const std::array<double, 3> & b) {
        for (std::size_t i = 0; i < a.size(); ++i) {
          if (!(a[i] == b[i] || (std::isnan(a[i]) && std::isnan(b[i])))) {
            return false;
          }
        }
        return true;
      };
    msg.settled = history_.size() >= 2U && same(values, history_[0]) && same(values, history_[1]);
    history_.push_front(values);
    while (history_.size() > 2U) {
      history_.pop_back();
    }

    std::lock_guard<std::mutex> lock(session_mutex_);
    const SessionInfo & s = session_;
    msg.acquisition_frame_rate_enable = s.acquisition_frame_rate_enable;
    msg.acquisition_frame_rate_hz = s.acquisition_frame_rate_hz;
    msg.exposure_auto = s.exposure_auto;
    msg.gain_auto = s.gain_auto;
    msg.balance_white_auto = s.balance_white_auto;
    msg.serial = s.serial;
    msg.model = s.model;
    msg.firmware_version = s.firmware_version;
    msg.adc_bit_depth = s.adc_bit_depth;
    msg.balance_ratio_red = s.balance_ratio_red;
    msg.balance_ratio_blue = s.balance_ratio_blue;
    msg.lut_enable = s.lut_enable;
    msg.white_level_dn = raw_white_level_dn_.load();
    msg.device_temperature_c = device_temperature_c_.load();
    msg.device_uptime_s = device_uptime_s_.load();
    msg.cfa_pattern = format.cfa;
    msg.bits_per_pixel = static_cast<std::uint8_t>(format.bits);
    msg.black_level_raw = s.black_level_raw;
    msg.hdr_output = s.hdr_output;
    msg.hdr_image_enhancement_enable = s.hdr_image_enhancement_enable;
    msg.hdr_digital_clamping_enable = s.hdr_digital_clamping_enable;
    msg.hdr_tuning_enable = s.hdr_tuning_enable;
    msg.hdr_channel_analog_gain_db = s.hdr_channel_analog_gain_db;
    msg.hdr_channel_exposure_time_us = s.hdr_channel_exposure_time_us;
    msg.balance_white_enable = s.balance_white_enable;
    msg.balance_ratio_green = s.balance_ratio_green;
    msg.color_transformation_enable = s.color_transformation_enable;
    msg.lut_tone_mapping = s.lut_tone_mapping;
    msg.reverse_x = s.reverse_x;
    msg.reverse_y = s.reverse_y;
    msg.sensor_shutter_mode = s.sensor_shutter_mode;
    return msg;
  }

  sensor_msgs::msg::CameraInfo BuildCameraInfo(const sensor_msgs::msg::Image & raw) const
  {
    sensor_msgs::msg::CameraInfo msg;
    msg.header = raw.header;
    msg.width = raw.width;
    msg.height = raw.height;
    msg.distortion_model = camera_info_distortion_model_;
    msg.d = camera_info_d_;
    std::copy(camera_info_k_.begin(), camera_info_k_.end(), msg.k.begin());
    std::copy(camera_info_r_.begin(), camera_info_r_.end(), msg.r.begin());
    std::copy(camera_info_p_.begin(), camera_info_p_.end(), msg.p.begin());
    return msg;
  }

  // GetImage polls in short slices so that a live change of a node that is locked while streaming
  // (HDROutput, PixelFormat, AcquisitionFrameRate on the TRI054S) can park this thread, stop the stream,
  // write and restart — see WithStreamStopped().
  void AcquisitionLoop()
  {
    std::optional<RawFormat> format = ParseRawFormat(pixel_format_);
    std::uint64_t last_frame_id = 0;
    auto last_frame_time = std::chrono::steady_clock::now();
    while (rclcpp::ok() && running_.load()) {
      {
        std::unique_lock<std::mutex> lock(stream_mutex_);
        if (pause_requested_) {
          paused_ = true;
          stream_cv_.notify_all();
          stream_cv_.wait(lock, [this]() {return !pause_requested_ || !running_.load();});
          paused_ = false;
          format = ParseRawFormat(pixel_format_);
          last_frame_id = 0;
          history_.clear();
          last_frame_time = std::chrono::steady_clock::now();
          continue;
        }
      }

      Arena::IImage * image = nullptr;
      try {
        image = device_->GetImage(200);
      } catch (const GenICam::TimeoutException &) {
        if (!trigger_armed_.load() && std::chrono::steady_clock::now() - last_frame_time >
          std::chrono::milliseconds(std::max(100, acquisition_timeout_ms_)))
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "No frame within %d ms (Failed waiting for NEW_BUFFER_DATA).", acquisition_timeout_ms_);
          ReportStreamStats();
        }
        continue;
      } catch (...) {
        const std::string exception_text = CurrentExceptionText();
        if (!running_.load()) {
          break;
        }
        if (!device_->IsConnected()) {
          RCLCPP_ERROR(
            get_logger(), "Stopping acquisition after fatal Arena stream error: %s (camera disconnected)",
            exception_text.c_str());
          running_.store(false);
          break;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Arena acquisition warning: %s", exception_text.c_str());
        continue;
      }
      last_frame_time = std::chrono::steady_clock::now();

      try {
        if (image->IsIncomplete()) {
          ++incomplete_frames_;
          ++window_incomplete_;
          const double filled = static_cast<double>(image->GetSizeFilled());
          const double payload = static_cast<double>(std::max<std::size_t>(1, image->GetPayloadSize()));
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Incomplete image received (%.1f%% of the payload). Packets are being lost on the way — "
            "too much traffic on the camera link?", 100.0 * filled / payload);
          device_->RequeueBuffer(image);
          ReportStreamStats();
          continue;
        }
        const rclcpp::Time host_stamp = now();
        const FrameInfo info = ReadFrameInfo(image);
        const rclcpp::Time stamp = ResolveStamp(info.timestamp_ns, info.exposure_us, host_stamp);
        if (last_frame_id != 0U && info.frame_id != last_frame_id + 1U) {
          window_gaps_ += info.frame_id > last_frame_id ? info.frame_id - last_frame_id - 1U : 1U;
        }
        last_frame_id = info.frame_id;

        const bool want_preview = publish_rgb_compressed_ && PreviewDue();
        sensor_msgs::msg::Image raw = BuildRawImage(image, *format, stamp, want_preview);
        device_->RequeueBuffer(image);
        image = nullptr;

        if (publish_metadata_) {
          metadata_pub_->publish(BuildMetadata(raw, *format, info, stamp));
        }
        if (publish_camera_info_) {
          camera_info_pub_->publish(BuildCameraInfo(raw));
        }
        if (publish_raw_) {
          raw_pub_->publish(std::move(raw));
        }
        ++window_published_;
        ReportStreamStats();
      } catch (...) {
        const std::string exception_text = CurrentExceptionText();
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "Image publish error: %s", exception_text.c_str());
        if (image != nullptr) {
          try {
            device_->RequeueBuffer(image);
          } catch (...) {
          }
        }
      }
    }
  }

  // Executor thread: park the acquisition thread, stop the stream, run `apply`, restart the stream.
  // Used for nodes that are locked while streaming. The camera keeps its settings; frames stop for
  // the time it takes (about half a second on the TRI054S).
  void WithStreamStopped(const std::function<void()> & apply, bool stop_stream = true)
  {
    {
      std::unique_lock<std::mutex> lock(stream_mutex_);
      pause_requested_ = true;
      if (!stream_cv_.wait_for(lock, std::chrono::seconds(5), [this]() {return paused_ || !running_.load();}) ||
        !running_.load())
      {
        pause_requested_ = false;
        stream_cv_.notify_all();
        throw std::runtime_error("the acquisition thread did not pause");
      }
    }
    std::string error;
    try {
      if (stop_stream) {
        device_->StopStream();
        stream_started_ = false;
      }
      apply();
    } catch (...) {
      error = CurrentExceptionText();
    }
    if (stop_stream) {
      try {
        device_->StartStream(static_cast<std::size_t>(stream_buffer_count_));
        stream_started_ = true;
      } catch (...) {
        const std::string text = CurrentExceptionText();
        error += (error.empty() ? "" : "; ") + std::string("restarting the stream failed: ") + text;
      }
    }
    {
      std::lock_guard<std::mutex> lock(stream_mutex_);
      pause_requested_ = false;
    }
    stream_cv_.notify_all();
    if (!error.empty()) {
      throw std::runtime_error(error);
    }
  }

  // ExposureTime with the frame rate fitted to it (exposure.auto_frame_rate): the TRI054S caps
  // ExposureTime at the frame period, and AcquisitionFrameRate is locked while streaming. The frame
  // rate is min(cap, 1 / (exposure + margin)), cap = parameter camera.AcquisitionFrameRate.
  // Call with the stream stopped (or not started yet).
  void ApplyExposureWithFrameRate(double exposure_us, std::optional<double> cap_override = std::nullopt)
  {
    SetBoolean(camera_map_, "AcquisitionFrameRateEnable", true);
    GenApi::INode * rate_node = FindNode(camera_map_, "AcquisitionFrameRate");
    GenApi::INode * exposure_node = FindNode(camera_map_, "ExposureTime");
    if (!Writable(rate_node) || !Writable(exposure_node)) {
      throw std::runtime_error("AcquisitionFrameRate / ExposureTime are not writable");
    }
    GenApi::CFloatPtr rate(rate_node);
    GenApi::CFloatPtr exposure(exposure_node);
    double cap = cap_override.value_or(std::numeric_limits<double>::infinity());
    rclcpp::Parameter cap_parameter;
    if (!cap_override && get_parameter("camera.AcquisitionFrameRate", cap_parameter) &&
      cap_parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      cap = cap_parameter.as_double();
    }
    const double fit = 1.0e6 / (exposure_us + frame_rate_margin_us_);
    const double target = std::clamp(std::min(cap, fit), rate->GetMin(), rate->GetMax());
    // Write order does not decide the TRI054S long-exposure black level (bistable, -9.7 DN at 1 s): writing
    // the lowest frame rate first, like the PTC scripts, made no difference (2026-10-20).
    if (exposure_us > exposure->GetMax()) {
      rate->SetValue(target);
      exposure->SetValue(exposure_us);
    } else {
      exposure->SetValue(exposure_us);
      rate->SetValue(target);
    }
    RCLCPP_INFO(
      get_logger(), "ExposureTime %.0f us with AcquisitionFrameRate %.3f Hz (cap %.3f Hz).",
      exposure->GetValue(), rate->GetValue(), cap);
  }

  // Same line format as flir_spinnaker_camera_node so DM_clipGUI's diagnostics can collect it.
  void ReportStreamStats()
  {
    if (stream_stats_interval_sec_ <= 0.0) {
      return;
    }
    const auto now_steady = std::chrono::steady_clock::now();
    const auto interval = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(stream_stats_interval_sec_));
    if (next_stats_report_.time_since_epoch().count() == 0) {
      next_stats_report_ = now_steady + interval;
      return;
    }
    if (now_steady < next_stats_report_) {
      return;
    }
    next_stats_report_ = now_steady + interval;
    const auto stream_value = [this](const char * name) {
        return static_cast<long>(ReadInteger(stream_map_, name).value_or(-1));
      };
    const bool trouble = window_incomplete_ > 0U || window_gaps_ > 0U;
    const auto text =
      "stream_stats: interval=%.0fs frames=%lu incomplete=%lu frame_id_gaps=%lu total_incomplete=%lu "
      "missed_packets=%ld resend_req=%ld lost_frames=%ld";
    if (trouble) {
      RCLCPP_WARN(get_logger(), text, stream_stats_interval_sec_,
        static_cast<unsigned long>(window_published_), static_cast<unsigned long>(window_incomplete_),
        static_cast<unsigned long>(window_gaps_), static_cast<unsigned long>(incomplete_frames_),
        stream_value("StreamMissedPacketCount"), stream_value("StreamResendRequestCount"),
        stream_value("StreamLostFrameCount"));
    } else {
      RCLCPP_INFO(get_logger(), text, stream_stats_interval_sec_,
        static_cast<unsigned long>(window_published_), static_cast<unsigned long>(window_incomplete_),
        static_cast<unsigned long>(window_gaps_), static_cast<unsigned long>(incomplete_frames_),
        stream_value("StreamMissedPacketCount"), stream_value("StreamResendRequestCount"),
        stream_value("StreamLostFrameCount"));
    }
    window_published_ = window_incomplete_ = window_gaps_ = 0U;
  }

  // ======================================================================================
  // Preview thread
  // ======================================================================================

  struct PreviewJob
  {
    std::vector<std::uint32_t> data32;
    std::vector<std::uint16_t> data16;
    std::vector<std::uint8_t> data8;
    int bits{16};
    int width{0};
    int height{0};
    std::string cfa;
    double white{65535.0};
    rclcpp::Time stamp;
  };

  bool PreviewDue()
  {
    std::lock_guard<std::mutex> lock(preview_mutex_);
    if (preview_has_job_) {
      return false;
    }
    const double rate = preview_settings_.max_rate_hz;
    return rate <= 0.0 ||
           std::chrono::steady_clock::now() - last_preview_submit_ >= std::chrono::duration<double>(1.0 / rate);
  }

  template<typename T>
  void SubmitPreview(
    const T * data, std::uint32_t width, std::uint32_t height, const std::string & cfa, double white,
    const rclcpp::Time & stamp)
  {
    if (cfa.empty()) {
      return;   // mono: no preview
    }
    std::lock_guard<std::mutex> lock(preview_mutex_);
    if (preview_has_job_) {
      return;
    }
    last_preview_submit_ = std::chrono::steady_clock::now();
    PreviewJob & job = preview_job_;
    job.width = static_cast<int>(width);
    job.height = static_cast<int>(height);
    job.cfa = cfa;
    job.stamp = stamp;
    const std::uint32_t configured_white = raw_white_level_dn_.load();
    job.white = configured_white > 0U ? static_cast<double>(configured_white) : white;
    const std::size_t pixels = static_cast<std::size_t>(width) * height;
    if constexpr (sizeof(T) == 1) {
      job.bits = 8;
      job.data8.assign(data, data + pixels);
    } else if constexpr (sizeof(T) == 2) {
      job.bits = 16;
      job.data16.assign(data, data + pixels);
    } else {
      job.bits = 32;
      job.data32.assign(data, data + pixels);
    }
    preview_has_job_ = true;
    preview_cv_.notify_one();
  }

  void PreviewLoop()
  {
    while (true) {
      PreviewJob job;
      raw_preview::Settings settings;
      {
        std::unique_lock<std::mutex> lock(preview_mutex_);
        preview_cv_.wait(lock, [this]() {return preview_has_job_ || preview_stop_ || !running_.load();});
        if (preview_stop_ || !running_.load()) {
          return;
        }
        std::swap(job, preview_job_);
        preview_has_job_ = false;
        settings = preview_settings_;
      }
      try {
        raw_preview::Cfa cfa = raw_preview::Cfa::kRGGB;
        raw_preview::ParseCfa(job.cfa, &cfa);
        const std::size_t stride = static_cast<std::size_t>(job.width);
        cv::Mat bgr;
        if (job.bits == 8) {
          bgr = raw_preview::Render(job.data8.data(), job.width, job.height, stride, cfa, job.white, settings, preview_state_);
        } else if (job.bits == 16) {
          bgr = raw_preview::Render(job.data16.data(), job.width, job.height, stride, cfa, job.white, settings, preview_state_);
        } else {
          bgr = raw_preview::Render(job.data32.data(), job.width, job.height, stride, cfa, job.white, settings, preview_state_);
        }
        if (bgr.empty()) {
          continue;
        }
        sensor_msgs::msg::CompressedImage msg;
        msg.header.stamp = job.stamp;
        msg.header.frame_id = frame_id_;
        msg.format = "jpeg";
        if (!cv::imencode(".jpg", bgr, msg.data, {cv::IMWRITE_JPEG_QUALITY, settings.jpeg_quality})) {
          throw std::runtime_error("OpenCV failed to encode the preview JPEG.");
        }
        preview_pub_->publish(std::move(msg));
      } catch (...) {
        const std::string exception_text = CurrentExceptionText();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Preview failed: %s", exception_text.c_str());
      }
    }
  }

  // ======================================================================================
  // Shutdown
  // ======================================================================================

  void Shutdown() noexcept
  {
    running_.store(false);
    {
      std::lock_guard<std::mutex> lock(stream_mutex_);
      pause_requested_ = false;
    }
    stream_cv_.notify_all();
    housekeeping_timer_.reset();
    if (acquisition_thread_.joinable()) {
      acquisition_thread_.join();
    }
    if (clock_thread_.joinable()) {
      clock_thread_.join();
    }
    {
      std::lock_guard<std::mutex> lock(preview_mutex_);
      preview_stop_ = true;
    }
    preview_cv_.notify_all();
    if (preview_thread_.joinable()) {
      preview_thread_.join();
    }
    try {
      if (device_ != nullptr && stream_started_) {
        device_->StopStream();
        stream_started_ = false;
      }
    } catch (...) {
      const std::string exception_text = CurrentExceptionText();
      RCLCPP_WARN(get_logger(), "StopStream failed during shutdown: %s", exception_text.c_str());
    }
    try {
      if (system_ != nullptr && device_ != nullptr) {
        system_->DestroyDevice(device_);
      }
    } catch (...) {
      const std::string exception_text = CurrentExceptionText();
      RCLCPP_WARN(get_logger(), "DestroyDevice failed during shutdown: %s", exception_text.c_str());
    }
    device_ = nullptr;
    camera_map_ = stream_map_ = tl_device_map_ = nullptr;
    try {
      if (system_ != nullptr) {
        Arena::CloseSystem(system_);
      }
    } catch (...) {
    }
    system_ = nullptr;
    control_callback_.reset();
  }

  // ---------- parameters ----------
  std::string camera_serial_;
  int camera_index_{0};
  std::string frame_id_;
  int acquisition_timeout_ms_{10000};
  int device_discovery_timeout_ms_{2000};
  int camera_init_max_attempts_{6};
  int camera_init_retry_delay_ms_{2000};
  std::string publisher_qos_reliability_;
  int publisher_qos_depth_{5};
  bool publish_raw_{true};
  bool publish_camera_info_{true};
  bool publish_metadata_{true};
  bool publish_rgb_compressed_{true};
  std::string pixel_format_;
  std::string raw24_container_{"uint32"};
  int raw12p_shift_{0};
  std::string buffer_handling_mode_;
  int stream_buffer_count_{10};
  bool chunk_data_enable_{true};
  std::vector<std::string> chunk_data_selectors_;
  std::string timestamp_mode_{"camera_latched"};
  std::string timestamp_exposure_latch_{"start"};
  double timestamp_latch_interval_sec_{2.0};
  double stream_stats_interval_sec_{30.0};
  double raw_balance_ratio_red_{kNaN};
  double raw_balance_ratio_green_{kNaN};
  double raw_balance_ratio_blue_{kNaN};
  std::int64_t raw_black_level_raw_{-1};
  std::atomic<std::uint32_t> raw_white_level_dn_{0U};   // effective (RefreshSessionInfo)
  std::int64_t raw_white_level_param_{-1};
  std::string nodemap_dump_dir_;
  std::string profile_path_;
  std::string lens_model_;
  double lens_f_number_{kNaN};
  double temperature_interval_sec_{1.0};
  bool force_ip_enable_{false};
  std::string force_ip_address_;
  std::string force_ip_subnet_mask_;
  std::string force_ip_gateway_;
  std::string camera_info_distortion_model_;
  std::vector<double> camera_info_d_;
  std::array<double, 9> camera_info_k_{};
  std::array<double, 9> camera_info_r_{};
  std::array<double, 12> camera_info_p_{};

  // ---------- camera ----------
  Arena::ISystem * system_{nullptr};
  Arena::IDevice * device_{nullptr};
  GenApi::INodeMap * camera_map_{nullptr};
  GenApi::INodeMap * stream_map_{nullptr};
  GenApi::INodeMap * tl_device_map_{nullptr};
  bool stream_started_{false};
  bool restart_for_locked_{true};
  std::string trigger_mode_{"off"};
  std::int64_t trigger_device_key_{0x4A53};
  std::int64_t trigger_group_key_{1};
  std::int64_t trigger_group_mask_{1};
  std::atomic<bool> trigger_armed_{false};   // TriggerMode On: no frames between commands is normal
  BufferPortMemo buffer_port_memo_;   // DumpNodeMap: does a node read chunk/event data (see ReadsBufferPort)
  bool auto_frame_rate_{false};
  double frame_rate_margin_us_{500.0};
  std::set<std::string> locked_while_streaming_;
  // acquisition thread parking (WithStreamStopped)
  std::mutex stream_mutex_;
  std::condition_variable stream_cv_;
  bool pause_requested_{false};
  bool paused_{false};
  std::unordered_map<std::string, ControlBinding> bindings_;
  std::vector<rclcpp::Parameter> pending_;
  std::set<std::string> chunk_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr control_callback_;

  // ---------- session / housekeeping ----------
  mutable std::mutex session_mutex_;
  SessionInfo session_;
  std::atomic<double> device_temperature_c_{kNaN};
  std::atomic<double> device_uptime_s_{kNaN};   // reboot detection (tools/bracket.py)
  std::chrono::steady_clock::time_point next_temperature_read_{};
  std::optional<std::chrono::steady_clock::time_point> nodemap_dump_due_;
  rclcpp::TimerBase::SharedPtr housekeeping_timer_;
  std::deque<std::array<double, 3>> history_;   // acquisition thread

  // ---------- clock mapping ----------
  std::mutex clock_mutex_;
  std::deque<ClockSample> clock_samples_;
  bool clock_valid_{false};
  bool clock_warned_{false};
  double clock_slope_{1.0};
  double clock_intercept_ns_{0.0};
  std::int64_t clock_origin_camera_ns_{0};

  // ---------- stats (acquisition thread) ----------
  std::uint64_t incomplete_frames_{0};
  std::uint64_t window_published_{0};
  std::uint64_t window_incomplete_{0};
  std::uint64_t window_gaps_{0};
  std::chrono::steady_clock::time_point next_stats_report_{};

  // ---------- publishers / threads ----------
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<lucid_raw_camera::msg::LucidMetadata>::SharedPtr metadata_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr preview_pub_;
  std::atomic<bool> running_{false};
  std::thread acquisition_thread_;
  std::thread clock_thread_;
  std::thread preview_thread_;

  // ---------- preview ----------
  std::mutex preview_mutex_;
  std::condition_variable preview_cv_;
  raw_preview::Settings preview_settings_;   // preview_mutex_
  raw_preview::State preview_state_;         // preview thread
  PreviewJob preview_job_;                   // preview_mutex_
  bool preview_has_job_{false};
  bool preview_stop_{false};
  std::chrono::steady_clock::time_point last_preview_submit_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<LucidRawCameraNode>();
    rclcpp::spin(node);
  } catch (...) {
    const std::string exception_text = CurrentExceptionText();
    std::cerr << "lucid_raw_camera_node failed: " << exception_text.c_str() << std::endl;
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
