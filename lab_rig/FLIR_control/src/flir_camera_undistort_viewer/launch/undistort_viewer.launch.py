from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _optional_override(context, name, parser=None):
    value = LaunchConfiguration(name).perform(context)
    if value == "":
        return None
    return parser(value) if parser is not None else value


def _build_node(context):
    parameter_overrides = {}

    for name in (
        "input_topic",
        "camera_info_topic",
        "output_topic",
        "input_qos_reliability",
        "camera_info_qos_reliability",
        "output_qos_reliability",
    ):
        value = _optional_override(context, name)
        if value is not None:
            parameter_overrides[name] = value

    for name in (
        "output_jpeg_quality",
        "output_png_compression_level",
        "input_qos_depth",
        "camera_info_qos_depth",
        "output_qos_depth",
    ):
        value = _optional_override(context, name, int)
        if value is not None:
            parameter_overrides[name] = value

    return [
        Node(
            package="flir_camera_undistort_viewer",
            executable="flir_camera_undistort_viewer_node",
            name="flir_camera_undistort_viewer",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                parameter_overrides,
            ],
        )
    ]


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("flir_camera_undistort_viewer"), "config", "undistort_viewer.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace for the undistort viewer node.",
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
                "camera_info_topic",
                default_value="",
                description="Override the CameraInfo topic.",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="",
                description="Override the undistorted compressed output topic.",
            ),
            DeclareLaunchArgument(
                "output_jpeg_quality",
                default_value="",
                description="Override the JPEG quality used when the input stream is JPEG.",
            ),
            DeclareLaunchArgument(
                "output_png_compression_level",
                default_value="",
                description="Override the PNG compression level used when the input stream is PNG.",
            ),
            DeclareLaunchArgument(
                "input_qos_reliability",
                default_value="",
                description="Override the compressed image subscription QoS reliability.",
            ),
            DeclareLaunchArgument(
                "input_qos_depth",
                default_value="",
                description="Override the compressed image subscription QoS depth.",
            ),
            DeclareLaunchArgument(
                "camera_info_qos_reliability",
                default_value="",
                description="Override the CameraInfo subscription QoS reliability.",
            ),
            DeclareLaunchArgument(
                "camera_info_qos_depth",
                default_value="",
                description="Override the CameraInfo subscription QoS depth.",
            ),
            DeclareLaunchArgument(
                "output_qos_reliability",
                default_value="",
                description="Override the undistorted compressed output QoS reliability.",
            ),
            DeclareLaunchArgument(
                "output_qos_depth",
                default_value="",
                description="Override the undistorted compressed output QoS depth.",
            ),
            OpaqueFunction(function=_build_node),
        ]
    )
