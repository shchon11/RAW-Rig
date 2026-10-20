#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include "flir_camera_calibration/charuco_board.hpp"

namespace
{

std::string NormalizeName(std::string value)
{
  std::string normalized;
  normalized.reserve(value.size());

  for (const unsigned char character : value) {
    if (std::isalnum(character) != 0) {
      normalized.push_back(static_cast<char>(std::tolower(character)));
    }
  }

  return normalized;
}

std::string TrimAscii(std::string value)
{
  const auto is_space = [](unsigned char character) {
      return std::isspace(character) != 0;
    };

  value.erase(
    value.begin(),
    std::find_if_not(value.begin(), value.end(), is_space));
  value.erase(
    std::find_if_not(value.rbegin(), value.rend(), is_space).base(),
    value.end());
  return value;
}

std::string StripMatchingQuotes(std::string value)
{
  if (value.size() >= 2U) {
    const bool is_double_quoted = value.front() == '"' && value.back() == '"';
    const bool is_single_quoted = value.front() == '\'' && value.back() == '\'';
    if (is_double_quoted || is_single_quoted) {
      return value.substr(1, value.size() - 2U);
    }
  }

  return value;
}

std::string EscapeYamlDoubleQuoted(std::string value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    if (character == '\\' || character == '"') {
      escaped.push_back('\\');
    }
    escaped.push_back(character);
  }
  return escaped;
}

std::size_t CountLeadingSpaces(const std::string & value)
{
  return static_cast<std::size_t>(
    std::distance(
      value.begin(),
      std::find_if(value.begin(), value.end(), [](char character) {
        return character != ' ';
      })));
}

std::optional<std::string> MatchYamlMapKey(const std::string & line)
{
  const std::string trimmed = TrimAscii(line);
  if (trimmed.empty() || trimmed[0] == '#' || trimmed.back() != ':') {
    return std::nullopt;
  }

  return StripMatchingQuotes(TrimAscii(trimmed.substr(0, trimmed.size() - 1U)));
}

bool IsYamlMapKey(const std::string & line, const char * key)
{
  const auto map_key = MatchYamlMapKey(line);
  return map_key.has_value() && *map_key == key;
}

std::string FormatDouble(double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(10) << value;
  return stream.str();
}

std::string CurrentUtcTimestamp()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm utc_time{};
#if defined(_WIN32)
  gmtime_s(&utc_time, &now_time);
#else
  gmtime_r(&now_time, &utc_time);
#endif

  std::ostringstream stream;
  stream << std::put_time(&utc_time, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

}  // namespace

namespace
{

constexpr auto kUiPollInterval = std::chrono::milliseconds(10);

}  // namespace

class FlirCameraCalibrationNode : public rclcpp::Node
{
public:
  FlirCameraCalibrationNode()
  : Node("flir_camera_calibration"),
    input_topic_(declare_parameter<std::string>("input_topic", "/image_rgb/compressed")),
    annotated_output_topic_(declare_parameter<std::string>(
        "annotated_output_topic",
        "/calibration/image_annotated/compressed")),
    output_yaml_path_(declare_parameter<std::string>(
        "output_yaml_path",
        "calibration/flir_camera_info.yaml")),
    camera_serial_(declare_parameter<std::string>("camera_serial", "")),
    camera_name_(declare_parameter<std::string>("camera_name", "")),
    frame_id_(declare_parameter<std::string>("frame_id", "")),
    sample_image_dir_(declare_parameter<std::string>(
        "sample_image_dir",
        "calibration/captures")),
    board_cols_(declare_parameter<int>("board_cols", 6)),
    board_rows_(declare_parameter<int>("board_rows", 5)),
    square_size_m_(declare_parameter<double>("square_size_m", 0.08)),
    min_calibration_frames_(declare_parameter<int>("min_calibration_frames", 15)),
    display_window_(declare_parameter<bool>("display_window", true)),
    window_name_(declare_parameter<std::string>("window_name", "FLIR Calibration")),
    preview_scale_(declare_parameter<double>("preview_scale", 1.0)),
    preview_max_width_(declare_parameter<int>("preview_max_width", 640)),
    preview_fast_check_(declare_parameter<bool>("preview_fast_check", true)),
    annotated_jpeg_quality_(declare_parameter<int>("annotated_jpeg_quality", 80)),
    input_qos_reliability_(declare_parameter<std::string>("input_qos_reliability", "best_effort")),
    input_qos_depth_(declare_parameter<int>("input_qos_depth", 10)),
    output_qos_reliability_(declare_parameter<std::string>("output_qos_reliability", "reliable")),
    output_qos_depth_(declare_parameter<int>("output_qos_depth", 10)),
    board_type_(declare_parameter<std::string>("board_type", "chessboard")),
    charuco_squares_x_(declare_parameter<int>("charuco_squares_x", 8)),
    charuco_squares_y_(declare_parameter<int>("charuco_squares_y", 7)),
    charuco_square_length_m_(declare_parameter<double>("charuco_square_length_m", 0.12)),
    charuco_marker_length_m_(declare_parameter<double>("charuco_marker_length_m", 0.09)),
    aruco_dictionary_name_(declare_parameter<std::string>("aruco_dictionary", "DICT_5X5_1000")),
    charuco_min_corners_(declare_parameter<int>("charuco_min_corners", 12)),
    board_min_sharpness_(declare_parameter<double>("board_min_sharpness", 150.0)),
    auto_capture_(declare_parameter<bool>("auto_capture", false)),
    auto_capture_target_frames_(declare_parameter<int>("auto_capture_target_frames", 25)),
    auto_capture_min_move_frac_(declare_parameter<double>("auto_capture_min_move_frac", 0.05)),
    auto_capture_min_interval_sec_(declare_parameter<double>("auto_capture_min_interval_sec", 0.3)),
    auto_capture_target_rms_(declare_parameter<double>("auto_capture_target_rms", 1.5)),
    auto_capture_max_frames_(declare_parameter<int>("auto_capture_max_frames", 50)),
    board_size_(board_cols_, board_rows_)
  {
    if (board_cols_ <= 0 || board_rows_ <= 0) {
      throw std::runtime_error("board_cols and board_rows must both be positive.");
    }

    if (square_size_m_ <= 0.0) {
      throw std::runtime_error("square_size_m must be positive.");
    }

    if (preview_scale_ <= 0.0) {
      throw std::runtime_error("preview_scale must be positive.");
    }

    const std::string normalized_board_type = NormalizeName(board_type_);
    if (normalized_board_type == "charuco") {
      use_charuco_ = true;
    } else if (normalized_board_type == "chessboard") {
      use_charuco_ = false;
    } else {
      throw std::runtime_error("board_type must be either 'chessboard' or 'charuco'.");
    }

    if (use_charuco_) {
      // Throws with a clear message on bad ChArUco geometry / dictionary.
      charuco_board_ = std::make_unique<flir_camera_calibration::CharucoBoardModel>(
        charuco_squares_x_,
        charuco_squares_y_,
        charuco_square_length_m_,
        charuco_marker_length_m_,
        aruco_dictionary_name_);
    }

    base_object_points_ = BuildObjectPoints();

    annotated_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      annotated_output_topic_,
      BuildQoS(output_qos_reliability_, output_qos_depth_));

    image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      input_topic_,
      BuildQoS(input_qos_reliability_, input_qos_depth_),
      std::bind(&FlirCameraCalibrationNode::OnCompressedImage, this, std::placeholders::_1));

    if (display_window_) {
      cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    }
    keyboard_timer_ = create_wall_timer(
      kUiPollInterval,
      std::bind(&FlirCameraCalibrationNode::OnKeyboardTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "Calibration node listening on '%s' in %s mode. Press space to capture when the board is "
      "detected, 'c' to calibrate, 'r' to reset samples, and 'q' to quit.",
      input_topic_.c_str(),
      use_charuco_ ? "ChArUco" : "chessboard");

    // Keep the frame bounds positive so the run can always terminate: a non-positive
    // max cap would let quality mode accumulate frames forever if the target is unmet.
    min_calibration_frames_ = std::max(1, min_calibration_frames_);
    auto_capture_max_frames_ = std::max(1, auto_capture_max_frames_);

    // A count/cap below min_calibration_frames would trigger auto-calibration that
    // then gets refused, stalling the run silently. Raise them to match.
    if (auto_capture_ && auto_capture_target_frames_ > 0 &&
      auto_capture_target_frames_ < min_calibration_frames_)
    {
      RCLCPP_WARN(
        get_logger(),
        "auto_capture_target_frames (%d) is below min_calibration_frames (%d); raising it to %d.",
        auto_capture_target_frames_, min_calibration_frames_, min_calibration_frames_);
      auto_capture_target_frames_ = min_calibration_frames_;
    }
    if (auto_capture_ && auto_capture_max_frames_ < min_calibration_frames_) {
      RCLCPP_WARN(
        get_logger(),
        "auto_capture_max_frames (%d) is below min_calibration_frames (%d); raising it to %d.",
        auto_capture_max_frames_, min_calibration_frames_, min_calibration_frames_);
      auto_capture_max_frames_ = min_calibration_frames_;
    }

    if (auto_capture_) {
      if (auto_capture_target_rms_ > 0.0) {
        RCLCPP_INFO(
          get_logger(),
          "Auto-capture ON (quality mode): distinct poses are captured automatically and the "
          "calibration is saved once RMS <= %.2f, or at the %d-frame cap. No key presses needed.",
          auto_capture_target_rms_, auto_capture_max_frames_);
      } else {
        RCLCPP_INFO(
          get_logger(),
          "Auto-capture ON (count mode): calibration is saved once %d frames are collected. "
          "No key presses needed.",
          auto_capture_target_frames_);
      }
    }
  }

  ~FlirCameraCalibrationNode() override
  {
    if (display_window_) {
      try {
        cv::destroyWindow(window_name_);
      } catch (...) {
      }
    }
  }

private:
  struct LatestFrameState
  {
    bool has_frame = false;
    bool board_detected = false;
    std::uint64_t received_sequence = 0;
    std::uint64_t processed_sequence = 0;
    sensor_msgs::msg::CompressedImage::ConstSharedPtr message;
    sensor_msgs::msg::CompressedImage::_header_type header;
    cv::Mat annotated_preview_bgr;
    cv::Size image_size;
  };

  // Compact descriptor of a detected board's image-space placement, used by
  // auto-capture to decide whether a frame shows a pose it has not seen yet.
  struct PoseSignature
  {
    cv::Point2f centroid;
    float scale = 0.0F;  // bounding-box diagonal of the detected corners
  };

  enum class CaptureResult
  {
    Stored,
    NotDetected,
    SizeMismatch,
    TooBlurry,
  };

  // Focus measure (variance of the Laplacian) over the board's bounding box in the
  // full-resolution frame. Motion blur drops this sharply; it is the dominant driver
  // of high calibration RMS, so a captured frame must clear board_min_sharpness_.
  double BoardSharpness(const cv::Mat & bgr_image, const std::vector<cv::Point2f> & points) const
  {
    if (points.size() < 4) {
      return -1.0;
    }
    float min_x = points.front().x;
    float max_x = points.front().x;
    float min_y = points.front().y;
    float max_y = points.front().y;
    for (const cv::Point2f & point : points) {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
    }
    cv::Rect roi(
      cv::Point(static_cast<int>(std::floor(min_x)), static_cast<int>(std::floor(min_y))),
      cv::Point(static_cast<int>(std::ceil(max_x)) + 1, static_cast<int>(std::ceil(max_y)) + 1));
    roi &= cv::Rect(0, 0, bgr_image.cols, bgr_image.rows);
    if (roi.width < 8 || roi.height < 8) {
      return -1.0;
    }

    cv::Mat gray;
    cv::cvtColor(bgr_image(roi), gray, cv::COLOR_BGR2GRAY);
    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    return stddev[0] * stddev[0];
  }

  std::string NormalizeQoSReliability(const std::string & value) const
  {
    const std::string normalized = NormalizeName(value);
    if (normalized == "besteffort") {
      return "best_effort";
    }

    return "reliable";
  }

  rclcpp::QoS BuildQoS(const std::string & reliability, int depth) const
  {
    rclcpp::QoS qos{rclcpp::KeepLast(static_cast<std::size_t>(std::max(1, depth)))};
    qos.durability_volatile();

    if (NormalizeQoSReliability(reliability) == "best_effort") {
      qos.best_effort();
    } else {
      qos.reliable();
    }

    return qos;
  }

  std::vector<cv::Point3f> BuildObjectPoints() const
  {
    std::vector<cv::Point3f> object_points;
    object_points.reserve(static_cast<std::size_t>(board_cols_ * board_rows_));

    for (int row = 0; row < board_rows_; ++row) {
      for (int col = 0; col < board_cols_; ++col) {
        object_points.emplace_back(
          static_cast<float>(col * square_size_m_),
          static_cast<float>(row * square_size_m_),
          0.0F);
      }
    }

    return object_points;
  }

  void OnCompressedImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(latest_frame_mutex_);
      latest_frame_.has_frame = true;
      latest_frame_.message = msg;
      latest_frame_.header = msg->header;
      ++latest_frame_.received_sequence;
    }
  }

  cv::Mat DecodeCompressedImage(const sensor_msgs::msg::CompressedImage & msg) const
  {
    if (msg.data.empty()) {
      return cv::Mat();
    }

    const cv::Mat encoded_buffer(
      1,
      static_cast<int>(msg.data.size()),
      CV_8UC1,
      const_cast<std::uint8_t *>(msg.data.data()));
    return cv::imdecode(encoded_buffer, cv::IMREAD_COLOR);
  }

  bool DetectChessboard(
    const cv::Mat & bgr_image,
    std::vector<cv::Point2f> & corners,
    bool fast_check) const
  {
    cv::Mat grayscale;
    cv::cvtColor(bgr_image, grayscale, cv::COLOR_BGR2GRAY);

    int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    if (fast_check) {
      flags |= cv::CALIB_CB_FAST_CHECK;
    }
    const bool detected = cv::findChessboardCorners(grayscale, board_size_, corners, flags);
    if (!detected) {
      corners.clear();
      return false;
    }

    cv::cornerSubPix(
      grayscale,
      corners,
      cv::Size(11, 11),
      cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));

    return true;
  }

  cv::Mat PreparePreviewImage(const cv::Mat & bgr_image) const
  {
    if (preview_max_width_ <= 0 || bgr_image.cols <= preview_max_width_) {
      return bgr_image;
    }

    const double scale = static_cast<double>(preview_max_width_) / static_cast<double>(bgr_image.cols);
    cv::Mat preview;
    cv::resize(bgr_image, preview, cv::Size(), scale, scale, cv::INTER_AREA);
    return preview;
  }

  void DrawOverlay(cv::Mat & image, bool board_detected)
  {
    const std::size_t capture_count = CapturedCount();
    const cv::Scalar status_color = board_detected ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 0, 255);
    DrawOverlayText(
      image,
      board_detected ? "Board detected" : "Board not detected",
      0,
      status_color);
    std::string count_text = "Captured frames: " + std::to_string(capture_count);
    if (auto_capture_) {
      if (auto_capture_target_rms_ > 0.0) {
        count_text += " / " + std::to_string(auto_capture_max_frames_);
        if (last_auto_rms_ >= 0.0) {
          std::ostringstream rms_ss;
          rms_ss << "  RMS " << std::fixed << std::setprecision(2) << last_auto_rms_
                 << " (<=" << auto_capture_target_rms_ << ")";
          count_text += rms_ss.str();
        }
      } else if (auto_capture_target_frames_ > 0) {
        count_text += " / " + std::to_string(auto_capture_target_frames_);
      }
      count_text += auto_calibrated_ ? "  [AUTO done]" : "  [AUTO]";
    }
    DrawOverlayText(image, count_text, 1, cv::Scalar(255, 255, 255));
    DrawOverlayText(
      image,
      auto_capture_ ? "auto-capture on   c:calib  r:reset  q:quit"
                    : "space:capture  c:calib  r:reset  q:quit",
      2,
      cv::Scalar(255, 255, 255));

    if (board_min_sharpness_ > 0.0) {
      std::ostringstream sharp_ss;
      sharp_ss << "Sharpness: ";
      if (last_board_sharpness_ >= 0.0) {
        sharp_ss << static_cast<int>(last_board_sharpness_);
      } else {
        sharp_ss << "-";
      }
      sharp_ss << " (>=" << static_cast<int>(board_min_sharpness_) << ", hold still)";
      const bool sharp_ok = last_board_sharpness_ >= board_min_sharpness_;
      DrawOverlayText(
        image, sharp_ss.str(), 3,
        sharp_ok ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 165, 255));
    }
  }

  void DrawOverlayText(
    cv::Mat & image,
    const std::string & text,
    int line_index,
    const cv::Scalar & color) const
  {
    constexpr int left_margin = 12;
    constexpr int top_margin = 22;
    constexpr int line_spacing = 24;
    constexpr double base_scale = 0.58;
    constexpr int foreground_thickness = 1;
    constexpr int shadow_thickness = 3;
    int baseline = 0;
    const cv::Size text_size = cv::getTextSize(
      text,
      cv::FONT_HERSHEY_SIMPLEX,
      base_scale,
      foreground_thickness,
      &baseline);
    const int available_width = std::max(1, image.cols - left_margin * 2);
    const double scale = text_size.width <= available_width ?
      base_scale :
      std::max(0.35, base_scale * static_cast<double>(available_width) / static_cast<double>(text_size.width));
    const cv::Point origin(left_margin, top_margin + line_index * line_spacing);
    cv::putText(
      image,
      text,
      origin,
      cv::FONT_HERSHEY_SIMPLEX,
      scale,
      cv::Scalar(0, 0, 0),
      shadow_thickness,
      cv::LINE_AA);
    cv::putText(
      image,
      text,
      origin,
      cv::FONT_HERSHEY_SIMPLEX,
      scale,
      color,
      foreground_thickness,
      cv::LINE_AA);
  }

  void PublishAnnotatedImage(
    const sensor_msgs::msg::CompressedImage::_header_type & header,
    const cv::Mat & annotated_bgr)
  {
    if (annotated_pub_->get_subscription_count() == 0U) {
      return;
    }

    std::vector<std::uint8_t> encoded;
    const std::vector<int> parameters = {
      cv::IMWRITE_JPEG_QUALITY,
      std::clamp(annotated_jpeg_quality_, 0, 100)
    };

    if (!cv::imencode(".jpg", annotated_bgr, encoded, parameters)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Failed to encode annotated calibration preview.");
      return;
    }

    sensor_msgs::msg::CompressedImage output;
    output.header = header;
    output.format = "jpeg";
    output.data = std::move(encoded);
    annotated_pub_->publish(std::move(output));
  }

  void OnKeyboardTimer()
  {
    ProcessLatestFrame();

    LatestFrameState snapshot;
    {
      std::lock_guard<std::mutex> lock(latest_frame_mutex_);
      snapshot = latest_frame_;
    }

    if (!display_window_) {
      return;
    }

    if (!snapshot.has_frame) {
      return;
    }

    cv::Mat display_image = snapshot.annotated_preview_bgr;
    if (display_image.empty()) {
      return;
    }

    if (std::abs(preview_scale_ - 1.0) > 1e-6) {
      cv::resize(
        snapshot.annotated_preview_bgr,
        display_image,
        cv::Size(),
        preview_scale_,
        preview_scale_,
      cv::INTER_LINEAR);
    }

    cv::imshow(window_name_, display_image);

    const int key = cv::waitKey(1) & 0xFF;
    if (key == 0xFF) {
      return;
    }

    switch (key) {
      case ' ':
        CaptureCurrentFrame();
        break;
      case 'c':
      case 'C':
        CalibrateAndSave();
        break;
      case 'r':
      case 'R':
        ResetCapturedFrames();
        break;
      case 'q':
      case 'Q':
      case 27:
        RCLCPP_INFO(get_logger(), "Calibration window requested shutdown.");
        rclcpp::shutdown();
        break;
      default:
        break;
    }
  }

  void ProcessLatestFrame()
  {
    sensor_msgs::msg::CompressedImage::ConstSharedPtr message;
    std::uint64_t sequence = 0;
    {
      std::lock_guard<std::mutex> lock(latest_frame_mutex_);
      if (latest_frame_.message == nullptr ||
        latest_frame_.processed_sequence == latest_frame_.received_sequence)
      {
        return;
      }
      message = latest_frame_.message;
      sequence = latest_frame_.received_sequence;
    }

    ProcessFrame(message, sequence);
  }

  // Detects the board in a full-resolution frame and, if valid and dimension-
  // consistent, appends it to the capture set. Stays silent on plain non-detection
  // so auto-capture can call it every frame; the manual path logs its own note.
  CaptureResult StoreCapture(const cv::Mat & full_resolution_bgr)
  {
    cv::Mat charuco_corners;
    cv::Mat charuco_ids;
    std::vector<cv::Point2f> full_resolution_corners;
    if (use_charuco_) {
      // Require a well-populated view. Sparse partial views (only a handful of
      // corners) make cv::aruco::calibrateCameraCharuco throw in its per-view PnP.
      const int min_corners =
        std::max(charuco_min_corners_, flir_camera_calibration::CharucoBoardModel::kMinCornersForPose);
      if (!charuco_board_->Detect(full_resolution_bgr, charuco_corners, charuco_ids) ||
        static_cast<int>(charuco_ids.total()) < min_corners)
      {
        return CaptureResult::NotDetected;
      }
    } else if (!DetectChessboard(full_resolution_bgr, full_resolution_corners, false)) {
      return CaptureResult::NotDetected;
    }

    // Reject motion-blurred frames: the single biggest driver of high RMS. Measured
    // over the actual detected board region so it tracks the board, not the scene.
    const std::vector<cv::Point2f> board_points =
      use_charuco_ ? charuco_board_->ImagePoints(charuco_corners) : full_resolution_corners;
    last_board_sharpness_ = BoardSharpness(full_resolution_bgr, board_points);
    if (board_min_sharpness_ > 0.0 && last_board_sharpness_ >= 0.0 &&
      last_board_sharpness_ < board_min_sharpness_)
    {
      return CaptureResult::TooBlurry;
    }

    const cv::Size image_size = full_resolution_bgr.size();
    if (calibration_image_size_.width == 0 || calibration_image_size_.height == 0) {
      calibration_image_size_ = image_size;
    } else if (calibration_image_size_ != image_size) {
      RCLCPP_WARN(
        get_logger(),
        "Image size changed from %dx%d to %dx%d. Capture skipped.",
        calibration_image_size_.width,
        calibration_image_size_.height,
        image_size.width,
        image_size.height);
      return CaptureResult::SizeMismatch;
    }

    if (use_charuco_) {
      captured_charuco_corners_.push_back(charuco_corners);
      captured_charuco_ids_.push_back(charuco_ids);
    } else {
      captured_image_points_.push_back(full_resolution_corners);
      captured_object_points_.push_back(base_object_points_);
    }

    if (!sample_image_dir_.empty()) {
      SaveCapturedImage(full_resolution_bgr, CapturedCount());
    }

    RCLCPP_INFO(
      get_logger(),
      "Captured calibration frame %zu. Need at least %d frames before calibration.",
      CapturedCount(),
      min_calibration_frames_);
    return CaptureResult::Stored;
  }

  void CaptureCurrentFrame()
  {
    sensor_msgs::msg::CompressedImage::ConstSharedPtr message;
    {
      std::lock_guard<std::mutex> lock(latest_frame_mutex_);
      message = latest_frame_.message;
    }

    if (message == nullptr) {
      RCLCPP_WARN(get_logger(), "No image has been received yet.");
      return;
    }

    const cv::Mat full_resolution_bgr = DecodeCompressedImage(*message);
    if (full_resolution_bgr.empty()) {
      RCLCPP_WARN(get_logger(), "Failed to decode the latest compressed image for capture.");
      return;
    }

    const CaptureResult result = StoreCapture(full_resolution_bgr);
    if (result == CaptureResult::NotDetected) {
      RCLCPP_WARN(
        get_logger(),
        "%s",
        use_charuco_
          ? "ChArUco board not detected (or too few corners) in the latest full-resolution frame. "
          "Capture skipped."
          : "Chessboard not detected in the latest full-resolution frame. Capture skipped.");
    } else if (result == CaptureResult::TooBlurry) {
      RCLCPP_WARN(
        get_logger(),
        "Board too blurry to capture (sharpness %.0f < %.0f). Hold the board still.",
        last_board_sharpness_, board_min_sharpness_);
    }
  }

  std::optional<PoseSignature> ComputeSignature(const std::vector<cv::Point2f> & points) const
  {
    if (points.size() < static_cast<std::size_t>(
        flir_camera_calibration::CharucoBoardModel::kMinCornersForPose))
    {
      return std::nullopt;
    }

    cv::Point2f sum(0.0F, 0.0F);
    float min_x = points.front().x;
    float max_x = points.front().x;
    float min_y = points.front().y;
    float max_y = points.front().y;
    for (const cv::Point2f & point : points) {
      sum += point;
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
    }

    PoseSignature signature;
    signature.centroid = sum / static_cast<float>(points.size());
    signature.scale = std::hypot(max_x - min_x, max_y - min_y);
    return signature;
  }

  bool IsPoseDistinct(const PoseSignature & signature, double threshold_px) const
  {
    for (const PoseSignature & seen : captured_signatures_) {
      const double centroid_distance = cv::norm(signature.centroid - seen.centroid);
      const double scale_distance = std::abs(static_cast<double>(signature.scale - seen.scale));
      if (centroid_distance < threshold_px && scale_distance < threshold_px) {
        return false;
      }
    }
    return true;
  }

  // Auto-capture: called for every processed frame. Captures a frame only when the
  // board is at a pose (image position + apparent size) it has not already banked,
  // rate-limited by a short interval. Optionally triggers calibration at the target.
  void MaybeAutoCapture(
    const std::vector<cv::Point2f> & preview_points,
    int preview_width,
    const cv::Mat & full_resolution_bgr)
  {
    if (auto_calibrated_) {
      return;
    }
    // Stop banking new frames once the effective cap is hit: the max-frame ceiling
    // in quality mode, or the fixed target in count mode.
    const int capture_cap =
      (auto_capture_target_rms_ > 0.0) ? auto_capture_max_frames_ : auto_capture_target_frames_;
    if (capture_cap > 0 && CapturedCount() >= static_cast<std::size_t>(capture_cap)) {
      return;
    }

    const std::optional<PoseSignature> signature = ComputeSignature(preview_points);
    if (!signature) {
      return;
    }

    const rclcpp::Time current_time = now();
    if (has_last_auto_capture_time_ &&
      (current_time - last_auto_capture_time_).seconds() < auto_capture_min_interval_sec_)
    {
      return;
    }

    const double threshold_px =
      auto_capture_min_move_frac_ * static_cast<double>(std::max(1, preview_width));
    if (!IsPoseDistinct(*signature, threshold_px)) {
      return;
    }

    // Preview detection can succeed on the downscaled image while full-res detection
    // fails; in that case StoreCapture reports it and we simply retry next frame.
    if (StoreCapture(full_resolution_bgr) != CaptureResult::Stored) {
      return;
    }

    captured_signatures_.push_back(*signature);
    last_auto_capture_time_ = current_time;
    has_last_auto_capture_time_ = true;

    MaybeFinishAutoCapture();
  }

  // Decides, after a new capture, whether auto-capture is done. Quality mode keeps
  // recalibrating and stops when RMS reaches the target (or at the frame cap, saving
  // the best result). Count mode calibrates once at the fixed frame target.
  void MaybeFinishAutoCapture()
  {
    if (auto_capture_target_rms_ > 0.0) {
      if (CapturedCount() < static_cast<std::size_t>(std::max(1, min_calibration_frames_))) {
        return;
      }

      CalibrationOutput out;
      const bool ok = ComputeCalibration(out, /*verbose=*/false);
      if (ok) {
        last_auto_rms_ = out.rms;
        // RMS is not monotonic in frame count (a late blurry pose can raise it), so
        // remember the lowest-RMS result to fall back to at the cap.
        if (!has_best_output_ || out.rms < best_output_.rms) {
          best_output_ = out;
          has_best_output_ = true;
        }
        RCLCPP_INFO(
          get_logger(),
          "Auto-capture: %zu frames, RMS %.3f (target <= %.2f).",
          CapturedCount(), out.rms, auto_capture_target_rms_);
        if (out.rms <= auto_capture_target_rms_) {
          RCLCPP_INFO(get_logger(), "Target RMS reached. Saving and stopping auto-capture.");
          SaveCalibrationResult(out);
          auto_calibrated_ = true;
          return;
        }
      }

      if (auto_capture_max_frames_ > 0 &&
        CapturedCount() >= static_cast<std::size_t>(auto_capture_max_frames_))
      {
        if (has_best_output_) {
          RCLCPP_WARN(
            get_logger(),
            "Reached the %d-frame cap; best RMS %.3f is still above target %.2f. Saving the best "
            "result.",
            auto_capture_max_frames_, best_output_.rms, auto_capture_target_rms_);
          SaveCalibrationResult(best_output_);
        } else {
          RCLCPP_WARN(
            get_logger(),
            "Reached the %d-frame cap but calibration never converged. Reset (r) and recapture "
            "with more varied, well-filled views.",
            auto_capture_max_frames_);
        }
        auto_calibrated_ = true;
      }
      return;
    }

    // Count mode: calibrate once at the fixed target.
    if (auto_capture_target_frames_ > 0 &&
      CapturedCount() >= static_cast<std::size_t>(auto_capture_target_frames_))
    {
      RCLCPP_INFO(
        get_logger(),
        "Auto-capture collected %zu frames. Calibrating and saving...",
        CapturedCount());
      CalibrationOutput out;
      if (!(ComputeCalibration(out, /*verbose=*/true) && SaveCalibrationResult(out))) {
        RCLCPP_WARN(
          get_logger(),
          "Auto-calibration did not complete at %zu frames. Reset (r) and recapture with more "
          "varied views.",
          CapturedCount());
      }
      auto_calibrated_ = true;
    }
  }

  std::size_t CapturedCount() const
  {
    return use_charuco_ ? captured_charuco_corners_.size() : captured_image_points_.size();
  }

  void SaveCapturedImage(const cv::Mat & image, std::size_t capture_index)
  {
    try {
      const std::filesystem::path output_dir(sample_image_dir_);
      const std::filesystem::path serial_output_dir =
        camera_serial_.empty() ? output_dir : output_dir / camera_serial_;
      std::filesystem::create_directories(serial_output_dir);

      std::ostringstream filename;
      filename << "capture_" << std::setfill('0') << std::setw(3) << capture_index << ".jpg";
      const std::filesystem::path image_path = serial_output_dir / filename.str();
      cv::imwrite(image_path.string(), image);
    } catch (const std::exception & exception) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to save captured calibration image to '%s': %s",
        sample_image_dir_.c_str(),
        exception.what());
    }
  }

  struct CalibrationOutput
  {
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    double rms = 0.0;
    double mean_error = 0.0;
  };

  // Runs the calibration on the current captures. Returns false (warning only when
  // verbose) if there are too few frames or the solver throws. Used both by the
  // manual 'c' key and by auto-capture's repeated quality probing, so it must be
  // side-effect free apart from filling `out`.
  bool ComputeCalibration(CalibrationOutput & out, bool verbose)
  {
    if (CapturedCount() < static_cast<std::size_t>(std::max(1, min_calibration_frames_))) {
      if (verbose) {
        RCLCPP_WARN(
          get_logger(),
          "Need at least %d captured frames before calibration. Current count: %zu",
          min_calibration_frames_,
          CapturedCount());
      }
      return false;
    }
    if (calibration_image_size_.width <= 0 || calibration_image_size_.height <= 0) {
      if (verbose) {
        RCLCPP_WARN(get_logger(), "Calibration image size is invalid.");
      }
      return false;
    }

    out.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    out.distortion_coefficients = cv::Mat();
    std::vector<cv::Mat> rotation_vectors;
    std::vector<cv::Mat> translation_vectors;

    // calibrateCamera / calibrateCameraCharuco throw cv::Exception on a
    // near-degenerate sample set. This runs in a wall-timer callback, so an
    // uncaught throw escapes spin() and kills the node. Fail soft instead.
    try {
      if (use_charuco_) {
        out.rms = charuco_board_->CalibrateCamera(
          captured_charuco_corners_,
          captured_charuco_ids_,
          calibration_image_size_,
          out.camera_matrix,
          out.distortion_coefficients,
          rotation_vectors,
          translation_vectors);
        out.mean_error = ComputeMeanReprojectionErrorCharuco(
          out.camera_matrix,
          out.distortion_coefficients,
          rotation_vectors,
          translation_vectors);
      } else {
        out.rms = cv::calibrateCamera(
          captured_object_points_,
          captured_image_points_,
          calibration_image_size_,
          out.camera_matrix,
          out.distortion_coefficients,
          rotation_vectors,
          translation_vectors);
        out.mean_error = ComputeMeanReprojectionError(
          out.camera_matrix,
          out.distortion_coefficients,
          rotation_vectors,
          translation_vectors);
      }
    } catch (const cv::Exception & exception) {
      if (verbose) {
        RCLCPP_WARN(
          get_logger(),
          "calibrateCamera failed on the captured samples (often too few or "
          "near-coplanar/low-variation views). Capture more varied board poses and "
          "retry. Details: %s",
          exception.what());
      }
      return false;
    }

    return true;
  }

  bool SaveCalibrationResult(const CalibrationOutput & out)
  {
    try {
      WriteCalibrationYaml(
        out.camera_matrix, out.distortion_coefficients, out.rms, out.mean_error);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        get_logger(),
        "Calibration succeeded but saving '%s' failed: %s",
        output_yaml_path_.c_str(),
        exception.what());
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Calibration finished with RMS %.6f and mean reprojection error %.6f. Saved to '%s'.",
      out.rms,
      out.mean_error,
      output_yaml_path_.c_str());
    return true;
  }

  void CalibrateAndSave()
  {
    CalibrationOutput out;
    if (ComputeCalibration(out, /*verbose=*/true)) {
      SaveCalibrationResult(out);
    }
  }

  double ComputeMeanReprojectionError(
    const cv::Mat & camera_matrix,
    const cv::Mat & distortion_coefficients,
    const std::vector<cv::Mat> & rotation_vectors,
    const std::vector<cv::Mat> & translation_vectors) const
  {
    double total_squared_error = 0.0;
    std::size_t total_point_count = 0U;

    for (std::size_t index = 0; index < captured_object_points_.size(); ++index) {
      std::vector<cv::Point2f> projected_points;
      cv::projectPoints(
        captured_object_points_[index],
        rotation_vectors[index],
        translation_vectors[index],
        camera_matrix,
        distortion_coefficients,
        projected_points);

      const double error = cv::norm(captured_image_points_[index], projected_points, cv::NORM_L2);
      total_squared_error += error * error;
      total_point_count += projected_points.size();
    }

    if (total_point_count == 0U) {
      return 0.0;
    }

    return std::sqrt(total_squared_error / static_cast<double>(total_point_count));
  }

  double ComputeMeanReprojectionErrorCharuco(
    const cv::Mat & camera_matrix,
    const cv::Mat & distortion_coefficients,
    const std::vector<cv::Mat> & rotation_vectors,
    const std::vector<cv::Mat> & translation_vectors) const
  {
    double total_squared_error = 0.0;
    std::size_t total_point_count = 0U;

    const std::size_t frame_count =
      std::min(captured_charuco_ids_.size(), rotation_vectors.size());
    for (std::size_t index = 0; index < frame_count; ++index) {
      const std::vector<cv::Point3f> object_points =
        charuco_board_->ObjectPointsForIds(captured_charuco_ids_[index]);
      const std::vector<cv::Point2f> image_points =
        charuco_board_->ImagePoints(captured_charuco_corners_[index]);
      if (object_points.empty() || object_points.size() != image_points.size()) {
        continue;
      }

      std::vector<cv::Point2f> projected_points;
      cv::projectPoints(
        object_points,
        rotation_vectors[index],
        translation_vectors[index],
        camera_matrix,
        distortion_coefficients,
        projected_points);

      const double error = cv::norm(image_points, projected_points, cv::NORM_L2);
      total_squared_error += error * error;
      total_point_count += projected_points.size();
    }

    if (total_point_count == 0U) {
      return 0.0;
    }

    return std::sqrt(total_squared_error / static_cast<double>(total_point_count));
  }

  std::vector<double> FlattenMatToDoubles(const cv::Mat & matrix) const
  {
    cv::Mat flattened = matrix.reshape(1, 1);
    cv::Mat as_double;
    flattened.convertTo(as_double, CV_64F);

    std::vector<double> values;
    values.reserve(as_double.total());

    for (int column = 0; column < as_double.cols; ++column) {
      values.push_back(as_double.at<double>(0, column));
    }

    return values;
  }

  std::string FormatYamlList(const std::vector<double> & values) const
  {
    std::ostringstream stream;
    stream << "[";
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (index > 0U) {
        stream << ", ";
      }
      stream << FormatDouble(values[index]);
    }
    stream << "]";
    return stream.str();
  }

  void WriteLegacyCalibrationYaml(
    const cv::Mat & camera_matrix,
    const cv::Mat & distortion_coefficients,
    double rms,
    double mean_error) const
  {
    const std::filesystem::path output_path(output_yaml_path_);
    if (output_path.has_parent_path()) {
      std::filesystem::create_directories(output_path.parent_path());
    }

    const std::vector<double> k = FlattenMatToDoubles(camera_matrix);
    const std::vector<double> d = FlattenMatToDoubles(distortion_coefficients);
    const std::vector<double> r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };
    const std::vector<double> p = {
      camera_matrix.at<double>(0, 0), 0.0, camera_matrix.at<double>(0, 2), 0.0,
      0.0, camera_matrix.at<double>(1, 1), camera_matrix.at<double>(1, 2), 0.0,
      0.0, 0.0, 1.0, 0.0
    };

    std::ofstream stream(output_path.string());
    if (!stream.is_open()) {
      throw std::runtime_error("Failed to open calibration output file: " + output_yaml_path_);
    }

    stream << "flir_camera:\n";
    stream << "  ros__parameters:\n";
    stream << "    camera_info.distortion_model: \"plumb_bob\"\n";
    stream << "    camera_info.d: " << FormatYamlList(d) << "\n";
    stream << "    camera_info.k: " << FormatYamlList(k) << "\n";
    stream << "    camera_info.r: " << FormatYamlList(r) << "\n";
    stream << "    camera_info.p: " << FormatYamlList(p) << "\n";
    stream << "calibration:\n";
    stream << "  generated_at_utc: \"" << CurrentUtcTimestamp() << "\"\n";
    stream << "  image_width: " << calibration_image_size_.width << "\n";
    stream << "  image_height: " << calibration_image_size_.height << "\n";
    if (use_charuco_) {
      stream << "  board_type: \"charuco\"\n";
      stream << "  charuco_squares_x: " << charuco_squares_x_ << "\n";
      stream << "  charuco_squares_y: " << charuco_squares_y_ << "\n";
      stream << "  charuco_square_length_m: " << FormatDouble(charuco_square_length_m_) << "\n";
      stream << "  charuco_marker_length_m: " << FormatDouble(charuco_marker_length_m_) << "\n";
      stream << "  aruco_dictionary: \"" << EscapeYamlDoubleQuoted(aruco_dictionary_name_) << "\"\n";
    } else {
      stream << "  board_type: \"chessboard\"\n";
      stream << "  board_cols: " << board_cols_ << "\n";
      stream << "  board_rows: " << board_rows_ << "\n";
      stream << "  square_size_m: " << FormatDouble(square_size_m_) << "\n";
    }
    stream << "  captured_frames: " << CapturedCount() << "\n";
    stream << "  rms_reprojection_error: " << FormatDouble(rms) << "\n";
    stream << "  mean_reprojection_error: " << FormatDouble(mean_error) << "\n";
  }

  std::vector<std::string> RenderSerialIndexedCalibrationEntry(
    const std::vector<double> & k,
    const std::vector<double> & d,
    const std::vector<double> & r,
    const std::vector<double> & p,
    double rms,
    double mean_error) const
  {
    std::vector<std::string> lines;
    const std::string camera_name = camera_name_.empty() ? camera_serial_ : camera_name_;

    lines.push_back("  \"" + EscapeYamlDoubleQuoted(camera_serial_) + "\":");
    lines.push_back("    camera_name: \"" + EscapeYamlDoubleQuoted(camera_name) + "\"");
    lines.push_back("    frame_id: \"" + EscapeYamlDoubleQuoted(frame_id_) + "\"");
    lines.push_back("    camera_info:");
    lines.push_back("      distortion_model: \"plumb_bob\"");
    lines.push_back("      d: " + FormatYamlList(d));
    lines.push_back("      k: " + FormatYamlList(k));
    lines.push_back("      r: " + FormatYamlList(r));
    lines.push_back("      p: " + FormatYamlList(p));
    lines.push_back("      binning_x: 0");
    lines.push_back("      binning_y: 0");
    lines.push_back("      roi:");
    lines.push_back("        x_offset: 0");
    lines.push_back("        y_offset: 0");
    lines.push_back("        height: 0");
    lines.push_back("        width: 0");
    lines.push_back("        do_rectify: false");
    lines.push_back("    calibration:");
    lines.push_back("      generated_at_utc: \"" + CurrentUtcTimestamp() + "\"");
    lines.push_back("      image_width: " + std::to_string(calibration_image_size_.width));
    lines.push_back("      image_height: " + std::to_string(calibration_image_size_.height));
    if (use_charuco_) {
      lines.push_back("      board_type: \"charuco\"");
      lines.push_back("      charuco_squares_x: " + std::to_string(charuco_squares_x_));
      lines.push_back("      charuco_squares_y: " + std::to_string(charuco_squares_y_));
      lines.push_back("      charuco_square_length_m: " + FormatDouble(charuco_square_length_m_));
      lines.push_back("      charuco_marker_length_m: " + FormatDouble(charuco_marker_length_m_));
      lines.push_back("      aruco_dictionary: \"" + EscapeYamlDoubleQuoted(aruco_dictionary_name_) + "\"");
    } else {
      lines.push_back("      board_type: \"chessboard\"");
      lines.push_back("      board_cols: " + std::to_string(board_cols_));
      lines.push_back("      board_rows: " + std::to_string(board_rows_));
      lines.push_back("      square_size_m: " + FormatDouble(square_size_m_));
    }
    lines.push_back("      captured_frames: " + std::to_string(CapturedCount()));
    lines.push_back("      rms_reprojection_error: " + FormatDouble(rms));
    lines.push_back("      mean_reprojection_error: " + FormatDouble(mean_error));
    return lines;
  }

  std::vector<std::string> ReadExistingYamlLines(const std::filesystem::path & path) const
  {
    std::vector<std::string> lines;
    std::ifstream input(path.string());
    if (!input.is_open()) {
      return lines;
    }

    std::string line;
    while (std::getline(input, line)) {
      lines.push_back(line);
    }
    return lines;
  }

  void WriteLines(const std::filesystem::path & output_path, const std::vector<std::string> & lines) const
  {
    std::ofstream stream(output_path.string());
    if (!stream.is_open()) {
      throw std::runtime_error("Failed to open calibration output file: " + output_yaml_path_);
    }

    for (const std::string & line : lines) {
      stream << line << "\n";
    }
  }

  void WriteSerialIndexedCalibrationYaml(
    const cv::Mat & camera_matrix,
    const cv::Mat & distortion_coefficients,
    double rms,
    double mean_error) const
  {
    const std::filesystem::path output_path(output_yaml_path_);
    if (output_path.has_parent_path()) {
      std::filesystem::create_directories(output_path.parent_path());
    }

    const std::vector<double> k = FlattenMatToDoubles(camera_matrix);
    const std::vector<double> d = FlattenMatToDoubles(distortion_coefficients);
    const std::vector<double> r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };
    const std::vector<double> p = {
      camera_matrix.at<double>(0, 0), 0.0, camera_matrix.at<double>(0, 2), 0.0,
      0.0, camera_matrix.at<double>(1, 1), camera_matrix.at<double>(1, 2), 0.0,
      0.0, 0.0, 1.0, 0.0
    };
    const std::vector<std::string> entry_lines =
      RenderSerialIndexedCalibrationEntry(k, d, r, p, rms, mean_error);

    std::vector<std::string> lines = ReadExistingYamlLines(output_path);
    const auto registry_iter = std::find_if(
      lines.begin(),
      lines.end(),
      [](const std::string & line) {
        return IsYamlMapKey(line, "camera_info_by_serial");
      });

    if (registry_iter == lines.end()) {
      std::vector<std::string> fresh_lines;
      fresh_lines.push_back("version: 2");
      fresh_lines.push_back("camera_info_by_serial:");
      fresh_lines.insert(fresh_lines.end(), entry_lines.begin(), entry_lines.end());
      WriteLines(output_path, fresh_lines);
      return;
    }

    const std::size_t registry_index =
      static_cast<std::size_t>(std::distance(lines.begin(), registry_iter));
    const std::size_t registry_indent = CountLeadingSpaces(lines[registry_index]);
    std::size_t registry_end = lines.size();
    std::optional<std::size_t> target_start;
    std::size_t target_end = lines.size();

    for (std::size_t index = registry_index + 1U; index < lines.size(); ++index) {
      const std::string trimmed = TrimAscii(lines[index]);
      if (trimmed.empty() || trimmed[0] == '#') {
        continue;
      }

      const std::size_t indent = CountLeadingSpaces(lines[index]);
      if (indent <= registry_indent) {
        registry_end = index;
        break;
      }

      if (indent != registry_indent + 2U) {
        continue;
      }

      const auto serial_key = MatchYamlMapKey(lines[index]);
      if (!serial_key.has_value() || *serial_key != camera_serial_) {
        continue;
      }

      target_start = index;
      target_end = registry_end;
      for (std::size_t end_index = index + 1U; end_index < lines.size(); ++end_index) {
        const std::string end_trimmed = TrimAscii(lines[end_index]);
        if (end_trimmed.empty() || end_trimmed[0] == '#') {
          continue;
        }

        const std::size_t end_indent = CountLeadingSpaces(lines[end_index]);
        if (end_indent <= registry_indent ||
          (end_indent == registry_indent + 2U && MatchYamlMapKey(lines[end_index]).has_value()))
        {
          target_end = end_index;
          break;
        }
      }
      break;
    }

    if (target_start.has_value()) {
      lines.erase(lines.begin() + static_cast<std::ptrdiff_t>(*target_start),
        lines.begin() + static_cast<std::ptrdiff_t>(target_end));
      lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(*target_start),
        entry_lines.begin(), entry_lines.end());
    } else {
      lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(registry_end),
        entry_lines.begin(), entry_lines.end());
    }

    WriteLines(output_path, lines);
  }

  void WriteCalibrationYaml(
    const cv::Mat & camera_matrix,
    const cv::Mat & distortion_coefficients,
    double rms,
    double mean_error) const
  {
    if (camera_serial_.empty()) {
      WriteLegacyCalibrationYaml(camera_matrix, distortion_coefficients, rms, mean_error);
      return;
    }

    WriteSerialIndexedCalibrationYaml(camera_matrix, distortion_coefficients, rms, mean_error);
  }

  void ResetCapturedFrames()
  {
    captured_image_points_.clear();
    captured_object_points_.clear();
    captured_charuco_corners_.clear();
    captured_charuco_ids_.clear();
    captured_signatures_.clear();
    has_last_auto_capture_time_ = false;
    auto_calibrated_ = false;
    last_auto_rms_ = -1.0;
    has_best_output_ = false;
    calibration_image_size_ = cv::Size();
    RCLCPP_INFO(get_logger(), "Captured calibration frames cleared.");
  }

  void ProcessFrame(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & message,
    std::uint64_t sequence)
  {
    if (message == nullptr) {
      return;
    }

    const cv::Mat full_resolution_bgr = DecodeCompressedImage(*message);
    if (full_resolution_bgr.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Failed to decode compressed image from '%s'.",
        input_topic_.c_str());
      return;
    }

    cv::Mat preview_bgr = PreparePreviewImage(full_resolution_bgr);
    cv::Mat annotated_preview = preview_bgr.clone();
    bool board_detected = false;
    std::vector<cv::Point2f> preview_points;
    if (use_charuco_) {
      cv::Mat preview_charuco_corners;
      cv::Mat preview_charuco_ids;
      board_detected =
        charuco_board_->Detect(preview_bgr, preview_charuco_corners, preview_charuco_ids);
      if (board_detected) {
        charuco_board_->DrawDetected(annotated_preview, preview_charuco_corners, preview_charuco_ids);
        preview_points = charuco_board_->ImagePoints(preview_charuco_corners);
      }
    } else {
      board_detected = DetectChessboard(preview_bgr, preview_points, preview_fast_check_);
      if (board_detected) {
        cv::drawChessboardCorners(annotated_preview, board_size_, preview_points, true);
      }
    }
    // Live sharpness of the board on the full-resolution frame, for the overlay's
    // "hold still" feedback. Preview corners are scaled up to full-res coordinates.
    if (board_detected && !preview_points.empty()) {
      const double scale = static_cast<double>(full_resolution_bgr.cols) /
        static_cast<double>(std::max(1, preview_bgr.cols));
      std::vector<cv::Point2f> full_points;
      full_points.reserve(preview_points.size());
      for (const cv::Point2f & point : preview_points) {
        full_points.emplace_back(
          point.x * static_cast<float>(scale), point.y * static_cast<float>(scale));
      }
      last_board_sharpness_ = BoardSharpness(full_resolution_bgr, full_points);
    } else {
      last_board_sharpness_ = -1.0;
    }

    DrawOverlay(annotated_preview, board_detected);
    PublishAnnotatedImage(message->header, annotated_preview);

    if (auto_capture_ && board_detected) {
      MaybeAutoCapture(preview_points, preview_bgr.cols, full_resolution_bgr);
    }

    std::lock_guard<std::mutex> lock(latest_frame_mutex_);
    if (latest_frame_.received_sequence != sequence) {
      return;
    }
    latest_frame_.has_frame = true;
    latest_frame_.board_detected = board_detected;
    latest_frame_.header = message->header;
    latest_frame_.processed_sequence = sequence;
    latest_frame_.annotated_preview_bgr = annotated_preview;
    latest_frame_.image_size = full_resolution_bgr.size();
  }

  std::string input_topic_;
  std::string annotated_output_topic_;
  std::string output_yaml_path_;
  std::string camera_serial_;
  std::string camera_name_;
  std::string frame_id_;
  std::string sample_image_dir_;
  int board_cols_;
  int board_rows_;
  double square_size_m_;
  int min_calibration_frames_;
  bool display_window_;
  std::string window_name_;
  double preview_scale_;
  int preview_max_width_;
  bool preview_fast_check_;
  int annotated_jpeg_quality_;
  std::string input_qos_reliability_;
  int input_qos_depth_;
  std::string output_qos_reliability_;
  int output_qos_depth_;
  std::string board_type_;
  int charuco_squares_x_;
  int charuco_squares_y_;
  double charuco_square_length_m_;
  double charuco_marker_length_m_;
  std::string aruco_dictionary_name_;
  int charuco_min_corners_;
  double board_min_sharpness_;
  bool auto_capture_;
  int auto_capture_target_frames_;
  double auto_capture_min_move_frac_;
  double auto_capture_min_interval_sec_;
  double auto_capture_target_rms_;
  int auto_capture_max_frames_;
  bool use_charuco_{false};
  std::unique_ptr<flir_camera_calibration::CharucoBoardModel> charuco_board_;
  cv::Size board_size_;
  cv::Size calibration_image_size_;
  std::vector<cv::Point3f> base_object_points_;
  std::vector<std::vector<cv::Point2f>> captured_image_points_;
  std::vector<std::vector<cv::Point3f>> captured_object_points_;
  std::vector<cv::Mat> captured_charuco_corners_;
  std::vector<cv::Mat> captured_charuco_ids_;
  std::vector<PoseSignature> captured_signatures_;
  rclcpp::Time last_auto_capture_time_;
  bool has_last_auto_capture_time_{false};
  bool auto_calibrated_{false};
  double last_auto_rms_{-1.0};
  double last_board_sharpness_{-1.0};
  CalibrationOutput best_output_;
  bool has_best_output_{false};

  std::mutex latest_frame_mutex_;
  LatestFrameState latest_frame_;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr annotated_pub_;
  rclcpp::TimerBase::SharedPtr keyboard_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FlirCameraCalibrationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
