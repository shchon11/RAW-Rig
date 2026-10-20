"""Bring up the whole rig: 8 FLIR cameras + the Ouster lidar in one launch.

This is a thin composition layer. It owns no node logic of its own -- it includes
multicam.launch.py (cameras, ptp4l, extrinsics TF) and the ouster_ros
driver.launch.py, and adds the pre-flight checks that only matter once the two
run together.

    ros2 launch flir_spinnaker_camera all_sensors.launch.py

Why the sensors do not fight each other:
  - Different NICs. Cameras are on enp3s0f1 (10 GbE, 192.168.1.0/24), the lidar
    on eno1 (1 GbE, 169.254.0.0/16 link-local). No shared bandwidth.
  - Same clock. The lidar runs TIME_FROM_ROS_TIME and camera header.stamp is host
    arrival time, so both read CLOCK_REALTIME. multicam.launch.py's ptp4l is a
    grandmaster on the camera NIC only -- no -s, no phc2sys -- so it never
    disciplines the system clock and never shifts one sensor relative to the other.

The one real coupling, and why _check_lidar_reachable exists:
  ouster_ros driver.launch.py registers an OnStateTransition handler that emits
  launch.events.Shutdown when os_driver reaches 'finalized' ("Failed to
  communicate with the sensor in a timely manner"). A Shutdown event is global to
  the launch service -- there is no way to scope it to one IncludeLaunchDescription.
  So an unreachable lidar takes all eight cameras down with it, a minute or two
  after they finished streaming.

  We cannot stop that mid-run, but the common case is the lidar being off or
  unplugged at launch time, and that we can catch up front: probe the sensor
  before returning any actions, and fail with an actionable message instead of
  starting cameras that are about to be killed. Set lidar_required:=false to
  downgrade the probe to a warning, or enable_lidar:=false to skip the lidar.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os
import socket


# The lidar answers its HTTP config API on 80. A TCP connect is a better liveness
# probe than ICMP: it proves the sensor is booted and serving, not just that some
# device holds the IP. Keep the timeout short -- this runs before anything starts.
_LIDAR_TCP_PORT = 80
_LIDAR_PROBE_TIMEOUT_S = 3.0


def _package_share(package: str) -> str:
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

    try:
        return get_package_share_directory(package)
    except PackageNotFoundError as exc:
        raise RuntimeError(
            f"Package '{package}' was not found. The lidar driver is expected in the ROS "
            "underlay (/opt/ros/humble). Source it first, or pass enable_lidar:=false to "
            "bring up the cameras alone."
        ) from exc


def _sensor_hostname_from_params(params_file: str) -> str:
    """Pull sensor_hostname out of the driver params so the probe knows what to poke.

    Parsed with the same line-oriented approach as _parse_cameras_file in
    multicam.launch.py rather than importing yaml, to keep the launch files
    dependency-free. Returns "" when the key is absent, which callers treat as
    "nothing to probe".
    """
    try:
        with open(params_file, "r", encoding="utf-8") as stream:
            for raw_line in stream:
                line = raw_line.split("#", 1)[0].strip()
                if not line.startswith("sensor_hostname:"):
                    continue
                value = line.split(":", 1)[1].strip()
                if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
                    value = value[1:-1]
                return value.strip()
    except OSError as exc:
        raise RuntimeError(f"Could not read lidar params file '{params_file}': {exc}") from exc
    return ""


def _check_lidar_reachable(hostname: str, required: bool) -> None:
    if not hostname:
        return

    try:
        with socket.create_connection((hostname, _LIDAR_TCP_PORT), _LIDAR_PROBE_TIMEOUT_S):
            pass
        return
    except OSError as exc:
        message = (
            f"Lidar at '{hostname}' did not answer on TCP {_LIDAR_TCP_PORT} "
            f"within {_LIDAR_PROBE_TIMEOUT_S:.0f}s ({exc}).\n"
            "  Check: the sensor is powered, and 'ip route get " + hostname + "' resolves\n"
            "  via the lidar NIC (eno1 here -- see docs/network_layout.md).\n"
            "  ouster_ros shuts down the *entire* launch when it cannot reach the sensor,\n"
            "  which would take the cameras down too, so this launch stops now instead.\n"
            "  Pass lidar_required:=false to start anyway, or enable_lidar:=false to skip it."
        )
        if required:
            raise RuntimeError(message)
        print(f"[all_sensors] WARNING: {message}")


def _build(context, *args, **kwargs):
    enable_cameras = LaunchConfiguration("enable_cameras").perform(context).strip().lower()
    enable_lidar = LaunchConfiguration("enable_lidar").perform(context).strip().lower()
    lidar_required = LaunchConfiguration("lidar_required").perform(context).strip().lower()

    cameras_on = enable_cameras in {"1", "true", "yes", "on"}
    lidar_on = enable_lidar in {"1", "true", "yes", "on"}
    required = lidar_required in {"1", "true", "yes", "on"}

    if not cameras_on and not lidar_on:
        raise RuntimeError(
            "Both enable_cameras and enable_lidar are false -- nothing to launch."
        )

    actions = []

    if lidar_on:
        params_file = LaunchConfiguration("lidar_params_file").perform(context).strip()
        if not params_file:
            params_file = os.path.join(
                _package_share("flir_spinnaker_camera"), "config", "lidar_driver_params.yaml"
            )
        if not os.path.isfile(params_file):
            raise RuntimeError(f"lidar_params_file does not exist: {params_file}")

        # Probe before the cameras are appended, so a dead lidar costs no camera startup.
        _check_lidar_reachable(_sensor_hostname_from_params(params_file), required)

        ouster_launch = os.path.join(_package_share("ouster_ros"), "launch", "driver.launch.py")
        # GroupAction, not a bare IncludeLaunchDescription. An include does NOT open
        # its own scope: launch_arguments are applied as SetLaunchConfiguration in the
        # *enclosing* scope and outlive the include. Both this file and
        # multicam.launch.py have an argument called params_file, so a bare include
        # left params_file pointing at the lidar YAML, and multicam's
        # DeclareLaunchArgument default is skipped once the configuration already
        # exists. The cameras then loaded the lidar YAML, _load_ros_parameters found
        # no flir_camera key, returned {}, and all 87 shared parameters were silently
        # dropped -- the cameras ran on C++ defaults, so ptp_action.rate_hz fell back
        # to 10.0 and the rig streamed at 10 Hz instead of 30 Hz.
        # GroupAction is scoped by default, which confines the assignment.
        actions.append(
            GroupAction(
                [
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(ouster_launch),
                        launch_arguments={
                            "params_file": params_file,
                            "ouster_ns": LaunchConfiguration("lidar_namespace"),
                            # ouster_ros defaults viz to True, which pulls up rviz2.
                            # Off by default here so it does not compete with the
                            # camera pipeline; pass lidar_viz:=true to get it back.
                            "viz": LaunchConfiguration("lidar_viz"),
                        }.items(),
                    )
                ]
            )
        )

    if cameras_on:
        multicam_launch = os.path.join(
            _package_share("flir_spinnaker_camera"), "launch", "multicam.launch.py"
        )
        # Only args that get retuned per run are forwarded. Everything else keeps
        # multicam.launch.py's own defaults -- that file stays the source of truth
        # for camera behaviour, this one must not fork its defaults.
        forwarded = {}
        for name in (
            "cameras_file",
            "params_file",
            "ptp_master_interface",
            "ptp_action_role_override",
            "publish_extrinsics_tf",
        ):
            value = LaunchConfiguration(f"camera_{name}").perform(context)
            if value != "":
                forwarded[name] = value

        # Scoped for the same reason as the lidar include above, and so that
        # multicam.launch.py's own declared defaults cannot leak back out either.
        actions.append(
            GroupAction(
                [
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(multicam_launch),
                        launch_arguments=forwarded.items(),
                    )
                ]
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_cameras",
                default_value="true",
                description="Include multicam.launch.py (8 FLIR cameras, ptp4l, extrinsics TF).",
            ),
            DeclareLaunchArgument(
                "enable_lidar",
                default_value="true",
                description="Include the ouster_ros driver.",
            ),
            DeclareLaunchArgument(
                "lidar_required",
                default_value="true",
                description=(
                    "Abort the launch when the lidar does not answer the pre-flight probe. "
                    "Set false to start anyway and only warn -- note that ouster_ros will "
                    "still shut the whole launch down, cameras included, if it ultimately "
                    "cannot talk to the sensor."
                ),
            ),
            DeclareLaunchArgument(
                "lidar_params_file",
                default_value="",
                description=(
                    "Ouster driver params. Empty means this package's "
                    "config/lidar_driver_params.yaml."
                ),
            ),
            DeclareLaunchArgument(
                "lidar_namespace",
                default_value="ouster",
                description="Namespace for the lidar topics (/ouster/... by default).",
            ),
            DeclareLaunchArgument(
                "lidar_viz",
                default_value="false",
                description="Start ouster_ros' rviz2. Off by default -- it competes with debayering.",
            ),
            # Camera pass-throughs. Empty means "leave multicam.launch.py's default".
            DeclareLaunchArgument("camera_cameras_file", default_value=""),
            DeclareLaunchArgument("camera_params_file", default_value=""),
            DeclareLaunchArgument("camera_ptp_master_interface", default_value=""),
            DeclareLaunchArgument("camera_ptp_action_role_override", default_value=""),
            DeclareLaunchArgument("camera_publish_extrinsics_tf", default_value=""),
            OpaqueFunction(function=_build),
        ]
    )
