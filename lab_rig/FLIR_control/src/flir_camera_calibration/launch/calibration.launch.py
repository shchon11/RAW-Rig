from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def _load_ros_parameters(path: str, node_name: str) -> dict:
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    return dict(data.get(node_name, {}).get("ros__parameters", {}))


def _parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"Expected a boolean launch value, got: {value}")


def _optional_override(context, name, parser=None):
    value = LaunchConfiguration(name).perform(context)
    if value == "":
        return None
    return parser(value) if parser is not None else value


def _build_node(context):
    parameter_overrides = {}

    for name in (
        "input_topic",
        "annotated_output_topic",
        "output_yaml_path",
        "camera_serial",
        "camera_name",
        "frame_id",
        "sample_image_dir",
        "window_name",
        "input_qos_reliability",
        "output_qos_reliability",
        "board_type",
        "aruco_dictionary",
    ):
        value = _optional_override(context, name)
        if value is not None:
            parameter_overrides[name] = value

    for name in ("display_window", "preview_fast_check", "auto_capture"):
        value = _optional_override(context, name, _parse_bool)
        if value is not None:
            parameter_overrides[name] = value

    for name in (
        "board_cols",
        "board_rows",
        "min_calibration_frames",
        "annotated_jpeg_quality",
        "input_qos_depth",
        "output_qos_depth",
        "charuco_squares_x",
        "charuco_squares_y",
        "charuco_min_corners",
        "auto_capture_target_frames",
        "auto_capture_max_frames",
    ):
        value = _optional_override(context, name, int)
        if value is not None:
            parameter_overrides[name] = value

    for name in (
        "square_size_m",
        "preview_scale",
        "charuco_square_length_m",
        "charuco_marker_length_m",
        "auto_capture_min_move_frac",
        "auto_capture_min_interval_sec",
        "auto_capture_target_rms",
        "board_min_sharpness",
    ):
        value = _optional_override(context, name, float)
        if value is not None:
            parameter_overrides[name] = value

    for name in ("preview_max_width",):
        value = _optional_override(context, name, int)
        if value is not None:
            parameter_overrides[name] = value

    # Load the YAML into a plain dict instead of passing the file path. A
    # parameter file is matched by fully-qualified node name, so once the node
    # runs under a non-empty namespace (FQN "/<ns>/flir_camera_calibration") the
    # file's fixed "flir_camera_calibration:" block no longer matches and every
    # parameter is silently dropped. A dict is applied to the node directly,
    # regardless of namespace. (Same approach as multicam_calibration.launch.py.)
    params_file = LaunchConfiguration("params_file").perform(context)
    shared_parameters = _load_ros_parameters(params_file, "flir_camera_calibration")

    return [
        Node(
            package="flir_camera_calibration",
            executable="flir_camera_calibration_node",
            name="flir_camera_calibration",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            parameters=[
                shared_parameters,
                parameter_overrides,
            ],
        )
    ]


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("flir_camera_calibration"), "config", "calibration.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace for the calibration node.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file,
                description="Path to the parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "input_topic",
                default_value="",
                description="Override the compressed input topic.",
            ),
            DeclareLaunchArgument(
                "annotated_output_topic",
                default_value="",
                description="Override the annotated compressed output topic.",
            ),
            DeclareLaunchArgument(
                "output_yaml_path",
                default_value="",
                description="Override the saved calibration YAML path.",
            ),
            DeclareLaunchArgument(
                "camera_serial",
                default_value="",
                description="Optional camera serial used as the key in serial-indexed calibration YAML.",
            ),
            DeclareLaunchArgument(
                "camera_name",
                default_value="",
                description="Optional logical camera name such as camera_center.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="",
                description="Optional camera optical frame id stored with the calibration entry.",
            ),
            DeclareLaunchArgument(
                "sample_image_dir",
                default_value="",
                description="Override the optional directory for captured calibration frames.",
            ),
            DeclareLaunchArgument(
                "board_cols",
                default_value="",
                description="Override the chessboard inner-corner column count.",
            ),
            DeclareLaunchArgument(
                "board_rows",
                default_value="",
                description="Override the chessboard inner-corner row count.",
            ),
            DeclareLaunchArgument(
                "square_size_m",
                default_value="",
                description="Override the chessboard square size in meters.",
            ),
            DeclareLaunchArgument(
                "board_type",
                default_value="",
                description="Override the board type: chessboard or charuco.",
            ),
            DeclareLaunchArgument(
                "charuco_squares_x",
                default_value="",
                description="Override the ChArUco board square count in X.",
            ),
            DeclareLaunchArgument(
                "charuco_squares_y",
                default_value="",
                description="Override the ChArUco board square count in Y.",
            ),
            DeclareLaunchArgument(
                "charuco_square_length_m",
                default_value="",
                description="Override the ChArUco square length in meters.",
            ),
            DeclareLaunchArgument(
                "charuco_marker_length_m",
                default_value="",
                description="Override the ChArUco marker length in meters.",
            ),
            DeclareLaunchArgument(
                "aruco_dictionary",
                default_value="",
                description="Override the ChArUco ArUco dictionary, e.g. DICT_5X5_1000.",
            ),
            DeclareLaunchArgument(
                "charuco_min_corners",
                default_value="",
                description="Min ChArUco corners to accept a capture (rejects sparse views).",
            ),
            DeclareLaunchArgument(
                "board_min_sharpness",
                default_value="",
                description="Min board-region Laplacian variance to accept a capture (rejects blur).",
            ),
            DeclareLaunchArgument(
                "auto_capture",
                default_value="",
                description="Auto-capture distinct poses without key presses (true/false).",
            ),
            DeclareLaunchArgument(
                "auto_capture_target_rms",
                default_value="",
                description="Quality mode: stop + save when RMS <= this (0 = use frame-count mode).",
            ),
            DeclareLaunchArgument(
                "auto_capture_max_frames",
                default_value="",
                description="Quality-mode frame cap: save the best and stop if target RMS unmet.",
            ),
            DeclareLaunchArgument(
                "auto_capture_target_frames",
                default_value="",
                description="Count-mode frames before auto-calibrating (used when target_rms<=0).",
            ),
            DeclareLaunchArgument(
                "auto_capture_min_move_frac",
                default_value="",
                description="Min distinct-pose distance as a fraction of preview width.",
            ),
            DeclareLaunchArgument(
                "auto_capture_min_interval_sec",
                default_value="",
                description="Min seconds between two auto-captures.",
            ),
            DeclareLaunchArgument(
                "min_calibration_frames",
                default_value="",
                description="Override the minimum number of captured frames before calibration.",
            ),
            DeclareLaunchArgument(
                "display_window",
                default_value="",
                description="Override whether to show the OpenCV preview window.",
            ),
            DeclareLaunchArgument(
                "window_name",
                default_value="",
                description="Override the OpenCV preview window title.",
            ),
            DeclareLaunchArgument(
                "preview_scale",
                default_value="",
                description="Override the preview scaling factor.",
            ),
            DeclareLaunchArgument(
                "preview_max_width",
                default_value="",
                description="Override the preview max width used for live detection.",
            ),
            DeclareLaunchArgument(
                "preview_fast_check",
                default_value="",
                description="Override whether to use FAST_CHECK for live preview chessboard detection.",
            ),
            DeclareLaunchArgument(
                "annotated_jpeg_quality",
                default_value="",
                description="Override the annotated preview JPEG quality.",
            ),
            DeclareLaunchArgument(
                "input_qos_reliability",
                default_value="",
                description="Override the input QoS reliability: best_effort or reliable.",
            ),
            DeclareLaunchArgument(
                "input_qos_depth",
                default_value="",
                description="Override the input QoS queue depth.",
            ),
            DeclareLaunchArgument(
                "output_qos_reliability",
                default_value="",
                description="Override the annotated output QoS reliability: best_effort or reliable.",
            ),
            DeclareLaunchArgument(
                "output_qos_depth",
                default_value="",
                description="Override the annotated output QoS queue depth.",
            ),
            OpaqueFunction(function=_build_node),
        ]
    )
