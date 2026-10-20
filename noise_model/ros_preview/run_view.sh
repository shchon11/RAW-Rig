#!/bin/bash
# LUCID preview node (5 Hz). Stop it (Ctrl-C) before running any PTC script: only one process can own the camera.
source /opt/ros/humble/setup.bash
source /home/ircv/jsh/lucid_ws/install/setup.bash
A=/home/ircv/ArenaSDK_Linux_x64
export LD_LIBRARY_PATH=$A/lib64:$A/GenICam/library/lib/Linux64_x64:$A/ffmpeg:$A/Metavision/lib:$LD_LIBRARY_PATH
exec ros2 run lucid_vision_driver arena_camera_node_exe --ros-args --params-file /home/ircv/jsh/imx490_ptc/ros/lucid_view.param.yaml
