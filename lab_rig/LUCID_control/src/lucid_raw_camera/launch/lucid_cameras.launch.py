"""Launch one lucid_raw_camera_node per camera in an inventory YAML.

Same contract as FLIR_control's multicam.launch.py, so DM_clipGUI can drive it:
  params_file   shared ros__parameters for every camera (first top-level key)
  cameras_file  <key>: ros__parameters: cameras: [{name, serial, namespace, frame_id, ...}]
Each camera runs as node 'lucid_camera' in its own single-level namespace. Optional per-camera keys:
publish_raw, lens_model, f_number. force_ip_* is ignored here: IPs are set before launch (GUI / gvcp).
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_READY_TEXT = b"Camera acquisition started"


def _parse_bool(value: str) -> bool:
    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"Expected a boolean launch value, got: {value}")


def _ros_parameters(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    for value in data.values():
        if isinstance(value, dict) and "ros__parameters" in value:
            return dict(value["ros__parameters"] or {})
    return {}


def _cameras(path: str) -> list:
    cameras = list(_ros_parameters(path).get("cameras") or [])
    for index, camera in enumerate(cameras):
        camera.setdefault("name", f"lucid{index}")
        camera.setdefault("namespace", camera["name"])
        camera.setdefault("frame_id", f"{camera['name']}_optical_frame")
        if not str(camera.get("serial", "")).strip():
            raise RuntimeError(f"Camera '{camera['name']}' is missing a serial in {path}")
    return cameras


def _overrides(camera: dict, profile_path: str) -> dict:
    overrides = {
        "camera_serial": str(camera["serial"]),
        "frame_id": str(camera["frame_id"]),
        "profile_path": profile_path,
        "network.force_ip.enable": False,
    }
    publish_raw = str(camera.get("publish_raw", "")).strip()
    if publish_raw:
        overrides["publish_raw"] = _parse_bool(publish_raw)
    lens_model = str(camera.get("lens_model", "")).strip()
    if lens_model:
        overrides["lens.model"] = lens_model
    f_number = str(camera.get("f_number", "")).strip()
    if f_number:
        overrides["lens.f_number"] = float(f_number)
    return overrides


def _chain(nodes: list, timeout: float) -> list:
    """Start node k+1 after node k logs 'Camera acquisition started' (or exits, or `timeout`)."""
    actions = []
    for current, following in zip(nodes, nodes[1:]):
        fired = [False]

        def start_following(*_args, following=following, fired=fired):
            if fired[0]:
                return []
            fired[0] = True
            return [following]

        def on_output(event, start_following=start_following):
            return start_following() if _READY_TEXT in event.text else []

        actions.append(RegisterEventHandler(OnProcessIO(target_action=current, on_stdout=on_output,
                                                        on_stderr=on_output)))
        actions.append(RegisterEventHandler(OnProcessExit(target_action=current, on_exit=start_following)))
        if timeout > 0.0:
            actions.append(RegisterEventHandler(OnProcessStart(
                target_action=current,
                on_start=lambda *_a, start_following=start_following: [TimerAction(
                    period=timeout, actions=[OpaqueFunction(function=lambda _c: start_following())])])))
    if nodes:
        actions.append(nodes[0])
    return actions


def _build(context):
    if not _parse_bool(LaunchConfiguration("enable_cameras").perform(context)):
        return []
    params_file = os.path.abspath(LaunchConfiguration("params_file").perform(context))
    cameras_file = LaunchConfiguration("cameras_file").perform(context)
    shared = _ros_parameters(params_file)
    nodes = [
        Node(
            package="lucid_raw_camera",
            executable="lucid_raw_camera_node",
            name="lucid_camera",
            namespace=str(camera["namespace"]),
            output="screen",
            parameters=[shared, _overrides(camera, params_file)],
        )
        for camera in _cameras(cameras_file)
    ]
    if _parse_bool(LaunchConfiguration("sequential_start").perform(context)):
        return _chain(nodes, float(LaunchConfiguration("sequential_start_timeout").perform(context)))
    return nodes


def generate_launch_description():
    share = FindPackageShare("lucid_raw_camera")
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=PathJoinSubstitution([share, "config", "lucid_camera_raw.yaml"]),
            description="Shared parameter file used by every LUCID camera."),
        DeclareLaunchArgument(
            "cameras_file", default_value=PathJoinSubstitution([share, "config", "lab_lucid_cameras.yaml"]),
            description="Camera inventory YAML with name, serial, namespace and frame_id."),
        DeclareLaunchArgument(
            "enable_cameras", default_value="true",
            description="Launch the cameras listed in cameras_file."),
        DeclareLaunchArgument(
            "sequential_start", default_value="true",
            description="Start each camera node after the previous one logs 'Camera acquisition started'."),
        DeclareLaunchArgument(
            "sequential_start_timeout", default_value="45.0",
            description="With sequential_start, seconds before starting the next node anyway (0 disables)."),
        OpaqueFunction(function=_build),
    ])
