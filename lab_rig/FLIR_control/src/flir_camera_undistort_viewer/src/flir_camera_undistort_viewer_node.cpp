#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

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

std::string NormalizeCompressedFormat(const std::string & value)
{
  const std::string normalized = NormalizeName(value);
  if (normalized.find("png") != std::string::npos) {
    return "png";
  }

  return "jpeg";
}

}  // namespace

class FlirCameraUndistortViewerNode : public rclcpp::Node
{
public:
  FlirCameraUndistortViewerNode()
  : Node("flir_camera_undistort_viewer"),
    input_topic_(declare_parameter<std::string>("input_topic", "/image_rgb/compressed")),
    camera_info_topic_(declare_parameter<std::string>("camera_info_topic", "/camera_info")),
    output_topic_(declare_parameter<std::string>(
        "output_topic",
        "/image_rgb/undistorted/compressed")),
    output_jpeg_quality_(declare_parameter<int>("output_jpeg_quality", 80)),
    output_png_compression_level_(declare_parameter<int>("output_png_compression_level", 3)),
    input_qos_reliability_(declare_parameter<std::string>("input_qos_reliability", "best_effort")),
    input_qos_depth_(declare_parameter<int>("input_qos_depth", 10)),
    camera_info_qos_reliability_(declare_parameter<std::string>(
        "camera_info_qos_reliability",
        "reliable")),
    camera_info_qos_depth_(declare_parameter<int>("camera_info_qos_depth", 20)),
    output_qos_reliability_(declare_parameter<std::string>("output_qos_reliability", "best_effort")),
    output_qos_depth_(declare_parameter<int>("output_qos_depth", 10))
  {
    image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      input_topic_,
      BuildQoS(input_qos_reliability_, input_qos_depth_),
      std::bind(&FlirCameraUndistortViewerNode::OnCompressedImage, this, std::placeholders::_1));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      BuildQoS(camera_info_qos_reliability_, camera_info_qos_depth_),
      std::bind(&FlirCameraUndistortViewerNode::OnCameraInfo, this, std::placeholders::_1));

    output_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      output_topic_,
      BuildQoS(output_qos_reliability_, output_qos_depth_));

    RCLCPP_INFO(
      get_logger(),
      "Undistort publisher listening on image '%s' and camera_info '%s', publishing '%s'.",
      input_topic_.c_str(),
      camera_info_topic_.c_str(),
      output_topic_.c_str());
  }

private:
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

  void OnCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    latest_camera_info_ = *msg;
    has_camera_info_ = true;
  }

  void OnCompressedImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
  {
    sensor_msgs::msg::CameraInfo camera_info;
    {
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      if (!has_camera_info_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "Waiting for CameraInfo on '%s'.",
          camera_info_topic_.c_str());
        return;
      }

      camera_info = latest_camera_info_;
    }

    if (!HasUsableCalibration(camera_info)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "CameraInfo on '%s' does not contain usable intrinsics yet.",
        camera_info_topic_.c_str());
      return;
    }

    const cv::Mat input_bgr = DecodeCompressedImage(*msg);
    if (input_bgr.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Failed to decode compressed image from '%s'.",
        input_topic_.c_str());
      return;
    }

    if ((camera_info.width > 0U && camera_info.height > 0U) &&
      (camera_info.width != static_cast<std::uint32_t>(input_bgr.cols) ||
      camera_info.height != static_cast<std::uint32_t>(input_bgr.rows)))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "CameraInfo size %ux%u differs from image size %dx%d. Using the image size for undistortion.",
        camera_info.width,
        camera_info.height,
        input_bgr.cols,
        input_bgr.rows);
    }

    EnsureUndistortMaps(camera_info, input_bgr.size());

    cv::Mat undistorted_bgr;
    cv::remap(
      input_bgr,
      undistorted_bgr,
      undistort_map1_,
      undistort_map2_,
      cv::INTER_LINEAR);

    sensor_msgs::msg::CompressedImage output_message;
    output_message.header = msg->header;
    if (!EncodeCompressedImage(msg->format, undistorted_bgr, output_message)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Failed to encode undistorted image for '%s'.",
        output_topic_.c_str());
      return;
    }

    output_pub_->publish(std::move(output_message));
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

  bool EncodeCompressedImage(
    const std::string & input_format,
    const cv::Mat & image,
    sensor_msgs::msg::CompressedImage & output_message) const
  {
    std::vector<std::uint8_t> encoded;
    const std::string normalized_format = NormalizeCompressedFormat(input_format);

    if (normalized_format == "png") {
      const std::vector<int> parameters = {
        cv::IMWRITE_PNG_COMPRESSION,
        std::clamp(output_png_compression_level_, 0, 9)
      };

      if (!cv::imencode(".png", image, encoded, parameters)) {
        return false;
      }

      output_message.format = "png";
      output_message.data = std::move(encoded);
      return true;
    }

    const std::vector<int> parameters = {
      cv::IMWRITE_JPEG_QUALITY,
      std::clamp(output_jpeg_quality_, 0, 100)
    };

    if (!cv::imencode(".jpg", image, encoded, parameters)) {
      return false;
    }

    output_message.format = "jpeg";
    output_message.data = std::move(encoded);
    return true;
  }

  bool HasUsableCalibration(const sensor_msgs::msg::CameraInfo & camera_info) const
  {
    return std::abs(camera_info.k[0]) > 1e-9 && std::abs(camera_info.k[4]) > 1e-9;
  }

  bool CameraInfoMatchesCached(
    const sensor_msgs::msg::CameraInfo & camera_info,
    const cv::Size & image_size) const
  {
    return maps_valid_ &&
           cached_image_size_ == image_size &&
           cached_distortion_model_ == camera_info.distortion_model &&
           cached_d_ == camera_info.d &&
           std::equal(camera_info.k.begin(), camera_info.k.end(), cached_k_.begin()) &&
           std::equal(camera_info.r.begin(), camera_info.r.end(), cached_r_.begin()) &&
           std::equal(camera_info.p.begin(), camera_info.p.end(), cached_p_.begin());
  }

  cv::Mat BuildCameraMatrix(const sensor_msgs::msg::CameraInfo & camera_info) const
  {
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        camera_matrix.at<double>(row, col) = camera_info.k[static_cast<std::size_t>(row * 3 + col)];
      }
    }
    return camera_matrix;
  }

  cv::Mat BuildDistortionCoefficients(const sensor_msgs::msg::CameraInfo & camera_info) const
  {
    cv::Mat distortion_coefficients(
      1,
      static_cast<int>(camera_info.d.size()),
      CV_64F);

    for (std::size_t index = 0; index < camera_info.d.size(); ++index) {
      distortion_coefficients.at<double>(0, static_cast<int>(index)) = camera_info.d[index];
    }

    return distortion_coefficients;
  }

  cv::Mat BuildRectificationMatrix(const sensor_msgs::msg::CameraInfo & camera_info) const
  {
    cv::Mat rectification_matrix = cv::Mat::eye(3, 3, CV_64F);
    const bool has_non_zero_rectification = std::any_of(
      camera_info.r.begin(),
      camera_info.r.end(),
      [](double value) { return std::abs(value) > 1e-9; });

    if (!has_non_zero_rectification) {
      return rectification_matrix;
    }

    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        rectification_matrix.at<double>(row, col) = camera_info.r[static_cast<std::size_t>(row * 3 + col)];
      }
    }

    return rectification_matrix;
  }

  cv::Mat BuildProjectionMatrix(const sensor_msgs::msg::CameraInfo & camera_info) const
  {
    const bool has_non_zero_projection =
      std::abs(camera_info.p[0]) > 1e-9 && std::abs(camera_info.p[5]) > 1e-9;

    if (!has_non_zero_projection) {
      return BuildCameraMatrix(camera_info);
    }

    cv::Mat projection_matrix = cv::Mat::eye(3, 3, CV_64F);
    projection_matrix.at<double>(0, 0) = camera_info.p[0];
    projection_matrix.at<double>(0, 1) = camera_info.p[1];
    projection_matrix.at<double>(0, 2) = camera_info.p[2];
    projection_matrix.at<double>(1, 0) = camera_info.p[4];
    projection_matrix.at<double>(1, 1) = camera_info.p[5];
    projection_matrix.at<double>(1, 2) = camera_info.p[6];
    projection_matrix.at<double>(2, 0) = camera_info.p[8];
    projection_matrix.at<double>(2, 1) = camera_info.p[9];
    projection_matrix.at<double>(2, 2) = camera_info.p[10];
    return projection_matrix;
  }

  void EnsureUndistortMaps(
    const sensor_msgs::msg::CameraInfo & camera_info,
    const cv::Size & image_size)
  {
    if (CameraInfoMatchesCached(camera_info, image_size)) {
      return;
    }

    const cv::Mat camera_matrix = BuildCameraMatrix(camera_info);
    const cv::Mat distortion_coefficients = BuildDistortionCoefficients(camera_info);
    const cv::Mat rectification_matrix = BuildRectificationMatrix(camera_info);
    const cv::Mat projection_matrix = BuildProjectionMatrix(camera_info);

    cv::initUndistortRectifyMap(
      camera_matrix,
      distortion_coefficients,
      rectification_matrix,
      projection_matrix,
      image_size,
      CV_16SC2,
      undistort_map1_,
      undistort_map2_);

    cached_image_size_ = image_size;
    cached_distortion_model_ = camera_info.distortion_model;
    cached_d_ = camera_info.d;
    std::copy(camera_info.k.begin(), camera_info.k.end(), cached_k_.begin());
    std::copy(camera_info.r.begin(), camera_info.r.end(), cached_r_.begin());
    std::copy(camera_info.p.begin(), camera_info.p.end(), cached_p_.begin());
    maps_valid_ = true;
  }

  std::string input_topic_;
  std::string camera_info_topic_;
  std::string output_topic_;
  int output_jpeg_quality_;
  int output_png_compression_level_;
  std::string input_qos_reliability_;
  int input_qos_depth_;
  std::string camera_info_qos_reliability_;
  int camera_info_qos_depth_;
  std::string output_qos_reliability_;
  int output_qos_depth_;

  std::mutex camera_info_mutex_;
  sensor_msgs::msg::CameraInfo latest_camera_info_;
  bool has_camera_info_{false};

  bool maps_valid_{false};
  cv::Size cached_image_size_;
  std::string cached_distortion_model_;
  std::vector<double> cached_d_;
  std::array<double, 9> cached_k_{};
  std::array<double, 9> cached_r_{};
  std::array<double, 12> cached_p_{};
  cv::Mat undistort_map1_;
  cv::Mat undistort_map2_;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr output_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FlirCameraUndistortViewerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
