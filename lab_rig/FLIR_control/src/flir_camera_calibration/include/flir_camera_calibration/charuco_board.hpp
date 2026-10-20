// Shared ChArUco board model used by both the intrinsic and extrinsic
// calibration nodes. The plain-chessboard path in each node is untouched; this
// header only adds the ChArUco branch that board_type:="charuco" selects.
//
// Targets the OpenCV 4.5.x aruco API (getPredefinedDictionary / CharucoBoard::create
// / interpolateCornersCharuco / calibrateCameraCharuco / estimatePoseCharucoBoard),
// which is what ships with ROS 2 Humble on Ubuntu 22.04.
#ifndef FLIR_CAMERA_CALIBRATION__CHARUCO_BOARD_HPP_
#define FLIR_CAMERA_CALIBRATION__CHARUCO_BOARD_HPP_

#include <cctype>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace flir_camera_calibration
{

// Maps a predefined-dictionary name (e.g. "DICT_5X5_1000") to its aruco enum.
// Accepts case- and separator-insensitive spellings ("dict5x5_1000", "5X5_1000").
inline int ArucoDictionaryFromName(const std::string & name)
{
  std::string key;
  key.reserve(name.size());
  for (const unsigned char character : name) {
    if (std::isalnum(character) != 0) {
      key.push_back(static_cast<char>(std::toupper(character)));
    }
  }

  static const std::map<std::string, int> kDictionaries = {
    {"DICT4X450", cv::aruco::DICT_4X4_50},
    {"DICT4X4100", cv::aruco::DICT_4X4_100},
    {"DICT4X4250", cv::aruco::DICT_4X4_250},
    {"DICT4X41000", cv::aruco::DICT_4X4_1000},
    {"DICT5X550", cv::aruco::DICT_5X5_50},
    {"DICT5X5100", cv::aruco::DICT_5X5_100},
    {"DICT5X5250", cv::aruco::DICT_5X5_250},
    {"DICT5X51000", cv::aruco::DICT_5X5_1000},
    {"DICT6X650", cv::aruco::DICT_6X6_50},
    {"DICT6X6100", cv::aruco::DICT_6X6_100},
    {"DICT6X6250", cv::aruco::DICT_6X6_250},
    {"DICT6X61000", cv::aruco::DICT_6X6_1000},
    {"DICT7X750", cv::aruco::DICT_7X7_50},
    {"DICT7X7100", cv::aruco::DICT_7X7_100},
    {"DICT7X7250", cv::aruco::DICT_7X7_250},
    {"DICT7X71000", cv::aruco::DICT_7X7_1000},
    {"DICTARUCOORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
  };

  const auto found = kDictionaries.find(key);
  if (found == kDictionaries.end()) {
    throw std::runtime_error(
            "Unknown aruco_dictionary '" + name +
            "'. Use a predefined name such as DICT_5X5_1000.");
  }
  return found->second;
}

// Holds the ChArUco board definition and detector, and exposes the few
// operations the calibration nodes need. Read-only after construction, so it is
// safe to call Detect/EstimatePose from the single-threaded UI timer callback.
class CharucoBoardModel
{
public:
  CharucoBoardModel(
    int squares_x,
    int squares_y,
    double square_length_m,
    double marker_length_m,
    const std::string & dictionary_name)
  {
    if (squares_x < 2 || squares_y < 2) {
      throw std::runtime_error("charuco_squares_x and charuco_squares_y must both be >= 2.");
    }
    if (square_length_m <= 0.0 || marker_length_m <= 0.0) {
      throw std::runtime_error("charuco square and marker length must be positive.");
    }
    if (marker_length_m >= square_length_m) {
      throw std::runtime_error(
              "charuco_marker_length_m must be smaller than charuco_square_length_m.");
    }

    dictionary_ = cv::aruco::getPredefinedDictionary(ArucoDictionaryFromName(dictionary_name));
    board_ = cv::aruco::CharucoBoard::create(
      squares_x,
      squares_y,
      static_cast<float>(square_length_m),
      static_cast<float>(marker_length_m),
      dictionary_);
    detector_params_ = cv::aruco::DetectorParameters::create();
  }

  // Total number of interior chessboard corners this board can yield.
  int TotalCorners() const
  {
    return static_cast<int>(board_->chessboardCorners.size());
  }

  // A ChArUco pose needs at least this many interpolated corners to be reliable.
  static constexpr int kMinCornersForPose = 4;

  // Detects the board in a BGR image. Returns true and fills charuco_corners
  // (CV_32FC2) and charuco_ids (CV_32SC1) when at least one interior corner is
  // interpolated. Both outputs are released when nothing is found.
  bool Detect(const cv::Mat & bgr_image, cv::Mat & charuco_corners, cv::Mat & charuco_ids) const
  {
    charuco_corners.release();
    charuco_ids.release();

    cv::Mat grayscale;
    cv::cvtColor(bgr_image, grayscale, cv::COLOR_BGR2GRAY);

    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<std::vector<cv::Point2f>> rejected;
    std::vector<int> marker_ids;
    cv::aruco::detectMarkers(
      grayscale, dictionary_, marker_corners, marker_ids, detector_params_, rejected);
    if (marker_ids.empty()) {
      return false;
    }

    const int interpolated = cv::aruco::interpolateCornersCharuco(
      marker_corners, marker_ids, grayscale, board_, charuco_corners, charuco_ids);
    return interpolated > 0 && !charuco_ids.empty();
  }

  void DrawDetected(cv::Mat & bgr_image, const cv::Mat & charuco_corners, const cv::Mat & charuco_ids) const
  {
    if (charuco_corners.total() > 0U) {
      cv::aruco::drawDetectedCornersCharuco(
        bgr_image, charuco_corners, charuco_ids, cv::Scalar(0, 220, 0));
    }
  }

  // Full intrinsic calibration from per-frame corner/id sets.
  double CalibrateCamera(
    const std::vector<cv::Mat> & all_charuco_corners,
    const std::vector<cv::Mat> & all_charuco_ids,
    const cv::Size & image_size,
    cv::Mat & camera_matrix,
    cv::Mat & distortion_coefficients,
    std::vector<cv::Mat> & rotation_vectors,
    std::vector<cv::Mat> & translation_vectors) const
  {
    return cv::aruco::calibrateCameraCharuco(
      all_charuco_corners,
      all_charuco_ids,
      board_,
      image_size,
      camera_matrix,
      distortion_coefficients,
      rotation_vectors,
      translation_vectors);
  }

  // Single-view board pose (board -> camera). Returns false when the pose could
  // not be recovered, mirroring solvePnP's contract in the chessboard path.
  bool EstimatePose(
    const cv::Mat & charuco_corners,
    const cv::Mat & charuco_ids,
    const cv::Mat & camera_matrix,
    const cv::Mat & distortion_coefficients,
    cv::Vec3d & rotation_vector,
    cv::Vec3d & translation_vector) const
  {
    return cv::aruco::estimatePoseCharucoBoard(
      charuco_corners,
      charuco_ids,
      board_,
      camera_matrix,
      distortion_coefficients,
      rotation_vector,
      translation_vector);
  }

  // 3D board-frame positions of the detected corners, ordered to match
  // ImagePoints(charuco_corners). Ids outside the board range are dropped.
  std::vector<cv::Point3f> ObjectPointsForIds(const cv::Mat & charuco_ids) const
  {
    const std::vector<cv::Point3f> & all_corners = board_->chessboardCorners;
    std::vector<cv::Point3f> object_points;
    const int count = static_cast<int>(charuco_ids.total());
    object_points.reserve(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index) {
      const int id = charuco_ids.at<int>(index);
      if (id >= 0 && id < static_cast<int>(all_corners.size())) {
        object_points.push_back(all_corners[static_cast<std::size_t>(id)]);
      }
    }
    return object_points;
  }

  std::vector<cv::Point2f> ImagePoints(const cv::Mat & charuco_corners) const
  {
    std::vector<cv::Point2f> image_points;
    const int count = static_cast<int>(charuco_corners.total());
    image_points.reserve(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index) {
      image_points.push_back(charuco_corners.at<cv::Point2f>(index));
    }
    return image_points;
  }

private:
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::CharucoBoard> board_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
};

}  // namespace flir_camera_calibration

#endif  // FLIR_CAMERA_CALIBRATION__CHARUCO_BOARD_HPP_
