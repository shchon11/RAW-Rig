# FLIR ROS Workspace

FLIR/Teledyne Spinnaker 카메라를 ROS 2 Humble에서 쓰기 위한 워크스페이스다.
카메라 스트리밍, intrinsic/extrinsic calibration, 왜곡 보정 스트림, ROS 2 bag
기록, image-only nuScenes export까지 한 저장소에서 처리한다.

이 문서는 처음 실행할 때 필요한 순서와 자주 쓰는 명령을 먼저 보여준다. 패키지별
세부 파라미터와 운영 노트는 각 패키지 README를 참조하면 된다.

## Quick Start

새 PC에서 처음 한 번:

```bash
sudo apt install python3-pip python3-colcon-common-extensions python3-rosdep linuxptp
python3 -m pip install --user -r requirements.txt
```

`rosdep`를 처음 쓰는 PC라면 한 번만 초기화한다.

```bash
sudo rosdep init
rosdep update
```

ROS 의존성 설치:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Spinnaker SDK 4.x는 [Teledyne/FLIR 공식 배포본](https://www.teledynevisionsolutions.com/support/support-center/software-firmware-downloads/iis/spinnaker-sdk-download/spinnaker-sdk--download-files/?pn=Spinnaker+SDK&vn=Spinnaker+SDK)을
설치해야 한다. 기본 위치가 아니면 직접 지정한다.

```bash
export SPINNAKER_ROOT=/opt/spinnaker
```

빌드와 실행:

```bash
source scripts/setup_flir_env.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py
```

영상 확인:

```bash
ros2 run rqt_image_view rqt_image_view
```

## Common Commands

단일 카메라:

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py
```

시리얼을 지정해서 단일 카메라 실행:

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py camera_serial:=12345678
```

멀티캠:

```bash
ros2 launch flir_spinnaker_camera multicam.launch.py
```

카메라 8대 + Ouster 라이다 전체:

```bash
ros2 launch flir_spinnaker_camera all_sensors.launch.py
```

intrinsic calibration:

```bash
ros2 launch flir_camera_calibration calibration.launch.py
```

카메라별 intrinsic calibration:

```bash
ros2 launch flir_camera_calibration multicam_calibration.launch.py camera_name:=camera_center
```

왜곡 보정 스트림:

```bash
ros2 launch flir_camera_undistort_viewer undistort_viewer.launch.py
```

멀티캠 왜곡 보정 스트림:

```bash
ros2 launch flir_camera_undistort_viewer multicam_undistort_viewer.launch.py
```

최신 bag을 nuScenes layout으로 변환:

```bash
python3 scripts/bag_to_nuscenes.py --overwrite
```

raw Bayer와 RGB를 함께 export:

```bash
python3 scripts/bag_to_nuscenes.py --raw-and-rgb --overwrite
```

변환 결과 확인:

```bash
python3 scripts/serve_nuscenes_images.py
```

## Repository Map

| Path | Purpose |
| --- | --- |
| `src/flir_spinnaker_camera` | Spinnaker 카메라 노드, 단일/멀티캠 launch, ForceIP, trigger, PTP, TF |
| `src/flir_camera_calibration` | 체스보드 기반 intrinsic/extrinsic calibration |
| `src/flir_camera_undistort_viewer` | `/camera_info`를 사용한 왜곡 보정 이미지 퍼블리시 |
| `scripts/bag_to_nuscenes.py` | ROS 2 bag을 image-only nuScenes dataset으로 변환 |
| `scripts/serve_nuscenes_images.py` | 변환된 nuScenes 이미지를 timestamp 순서로 확인 |
| `scripts/multicam_param_set.py` | 모든 camera namespace에 같은 ROS parameter 적용 |
| `calibration/` | calibration YAML 산출물 위치 |
| `bags/` | 로컬 ROS 2 bag 저장 위치, gitignore 처리 |
| `nuscenes_export/` | 변환 결과 위치, gitignore 처리 |

패키지별 상세 문서:

- [`src/flir_spinnaker_camera/README.md`](src/flir_spinnaker_camera/README.md)
- [`src/flir_camera_calibration/README.md`](src/flir_camera_calibration/README.md)
- [`src/flir_camera_undistort_viewer/README.md`](src/flir_camera_undistort_viewer/README.md)

## Environment Notes

`scripts/setup_flir_env.bash`는 아래를 한 번에 맞춘다.

- ROS 2 Humble source
- `/opt/spinnaker`가 있으면 `SPINNAKER_ROOT` 설정
- `install/setup.bash` source
- `FLIR_ROS_WS` export

GigE 카메라 NIC와 PTP 권한은 helper로 맞출 수 있다.

```bash
scripts/setup_camera_nic.bash --interface enp5s0 --host-cidr 192.168.1.10/24
```

## Camera Streaming

`flir_spinnaker_camera`가 퍼블리시하는 기본 토픽:

- `/image_raw`
- `/camera_info`
- `/image_raw/metadata`
- `/image_rgb/compressed`

멀티캠에서는 namespace가 붙는다.

- `/camera_center/image_rgb/compressed`
- `/camera_center/camera_info`
- `/camera_front_right/image_rgb/compressed`
- `/camera_front_right/camera_info`

`/image_raw`는 raw/mono/Bayer 경로이고, `/image_rgb/compressed`는 host에서 RGB로
변환한 뒤 JPEG 또는 PNG로 압축한 결과다. 원본 장치 timestamp는
`/image_raw/metadata.camera_timestamp_ns`에 남는다.

## Multicam Setup

멀티캠 장치 매핑은 `src/flir_spinnaker_camera/config/multicam_cameras.yaml`에서
serial 기준으로 관리한다. 현재 rig의 8대 구성:

| 카메라 (namespace) | Serial | IP | MAC |
| --- | --- | --- | --- |
| `camera_center` | 25415248 | 192.168.1.1 | 2CDDA383CE50 |
| `camera_front_right` | 26076473 | 192.168.1.2 | 2CDDA38DE539 |
| `camera_front_left` | 26076477 | 192.168.1.3 | 2CDDA38DE53D |
| `camera_side_right_1` | 25415254 | 192.168.1.4 | 2CDDA383CE56 |
| `camera_side_right_2` | 26076472 | 192.168.1.5 | 2CDDA38DE538 |
| `camera_side_left_1` | 25415250 | 192.168.1.6 | 2CDDA383CE52 |
| `camera_side_left_2` | 25415249 | 192.168.1.7 | 2CDDA383CE51 |
| `camera_rear` | 26076003 | 192.168.1.8 | 2CDDA38DE363 |

`camera_center`가 PTP scheduled action의 sender이고 나머지는 receiver다.

```yaml
flir_multicam:
  ros__parameters:
    cameras:
      - name: "camera_center"
        serial: "25415248"
        namespace: "camera_center"
        frame_id: "camera_center_optical_frame"
        ptp_action_role: "sender"
        force_ip_address: "192.168.1.1"
      - name: "camera_front_right"
        serial: "26076473"
        namespace: "camera_front_right"
        frame_id: "camera_front_right_optical_frame"
        ptp_action_role: "receiver"
        force_ip_address: "192.168.1.2"
```

연결된 카메라를 감지해서 inventory YAML을 갱신하려면:

```bash
ros2 launch flir_spinnaker_camera multicam.launch.py auto_update_cameras_file:=true
```

카메라별 ForceIP, GPIO hardware trigger, PTP scheduled action trigger, extrinsic TF
설정은 `src/flir_spinnaker_camera/README.md`에 모아 두었다.

## Lidar (Ouster OS-2-128)

라이다는 이 워크스페이스에서 빌드하지 않는다. `ouster_ros`는 ROS underlay
(`/opt/ros/humble`)에 이미 설치되어 있고, overlay가 underlay를 가리지 않으므로
`source scripts/setup_flir_env.bash` 후 그대로 쓸 수 있다.

`all_sensors.launch.py`가 `multicam.launch.py`와 ouster 드라이버를 함께 띄운다.

```bash
# 전체 (카메라 8대 + 라이다)
ros2 launch flir_spinnaker_camera all_sensors.launch.py

# 라이다만
ros2 launch flir_spinnaker_camera all_sensors.launch.py enable_cameras:=false

# 카메라만 (multicam.launch.py와 동일)
ros2 launch flir_spinnaker_camera all_sensors.launch.py enable_lidar:=false
```

| 인자 | 기본값 | 설명 |
|---|---|---|
| `enable_cameras` / `enable_lidar` | `true` | 각 서브시스템 포함 여부 |
| `lidar_required` | `true` | 사전 점검에서 라이다가 응답 없으면 런치 중단 |
| `lidar_params_file` | (패키지 config) | ouster 드라이버 파라미터 경로 |
| `lidar_namespace` | `ouster` | 라이다 토픽 네임스페이스 |
| `lidar_viz` | `false` | ouster rviz2 동시 실행 |

카메라 인자는 `camera_` 접두사로 전달한다
(예: `camera_ptp_master_interface:=enp3s0f1`). 비워 두면 `multicam.launch.py`의
기본값이 그대로 쓰인다 — 카메라 동작의 기준은 여전히 그 파일이다.

**두 센서가 서로 간섭하지 않는 이유:**

- **NIC가 다르다.** 카메라는 `enp3s0f1`(10 GbE, 192.168.1.0/24), 라이다는
  `eno1`(1 GbE, 169.254.0.0/16). 대역폭을 공유하지 않는다.
- **시계가 같다.** 라이다는 `TIME_FROM_ROS_TIME`, 카메라 `header.stamp`는 호스트
  도착 시각이라 둘 다 시스템 시계를 읽는다. `multicam.launch.py`가 띄우는 `ptp4l`은
  카메라 NIC에 grandmaster로만 붙고 `-s`도 `phc2sys`도 없어 시스템 시계를 조정하지
  않는다. 즉 PTP가 두 센서의 상대 시각을 흔들지 않는다.

**주의 — 라이다 장애가 카메라까지 내린다.** ouster의 `driver.launch.py`는 센서와
통신하지 못하면 `launch.events.Shutdown`을 발생시키는데, 이 이벤트는 런치 전체에
적용되어 카메라 8대도 같이 종료된다. `IncludeLaunchDescription` 단위로 막을 방법이
없다. 그래서 `all_sensors.launch.py`는 **아무 노드도 띄우기 전에** 라이다 TCP 80
포트를 먼저 확인하고, 응답이 없으면 카메라를 시작하지 않은 채 중단한다.
`lidar_required:=false`로 경고만 남기게 바꿀 수 있지만, 실제로 통신에 실패하면
결국 전체가 내려가는 것은 동일하다.

라이다 파라미터는 `src/flir_spinnaker_camera/config/lidar_driver_params.yaml`이
기준이다. 원래 `~/lidar_ws/config/driver_params.yaml`에 있던 것을 워크스페이스 밖
경로 의존을 없애려고 리포 안으로 옮겼으므로, `~/lidar_ws` 쪽을 고쳐도 반영되지 않는다.

> `~/flir_ouster_ws`는 별개의 워크스페이스다. upstream `spinnaker_camera_driver`를
> 쓰고 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`를 강제하므로, 이 리포와 RMW가 달라
> 섞어서 실행하면 토픽이 서로 보이지 않는다. 둘 중 하나만 쓸 것.

## Calibration

intrinsic calibration은 OpenCV 창에서 진행한다.

- `space`: 현재 체스보드 샘플 캡처
- `c`: calibration 실행 및 YAML 저장
- `r`: 샘플 초기화
- `q` 또는 `Esc`: 종료

결과는 기본적으로 `calibration/flir_camera_info.yaml`에 저장된다. 카메라 노드에
반영하려면 config 또는 launch argument로 YAML path를 넘긴다.

```yaml
camera_info.yaml_path: "calibration/flir_camera_info.yaml"
```

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py \
  camera_info_yaml_path:=calibration/flir_camera_info.yaml
```

멀티캠 extrinsic calibration:

```bash
ros2 launch flir_camera_calibration multicam_extrinsic_calibration.launch.py
```

결과는 `calibration/flir_camera_extrinsics.yaml`에 저장되고, multicam launch가
기본적으로 `/tf_static`에 publish한다.

## Recording Bags

nuScenes 변환만 할 거면 compressed RGB, `camera_info`, metadata를 같이 기록한다.

```bash
mkdir -p bags

CAMERAS=(camera_center camera_front_right camera_front_left \
  camera_side_right_1 camera_side_right_2 \
  camera_side_left_1 camera_side_left_2 camera_rear)

ros2 bag record \
  -o bags/flir_multicam_$(date +%Y%m%d_%H%M%S) \
  $(for cam in "${CAMERAS[@]}"; do
      echo "/$cam/image_rgb/compressed /$cam/camera_info /$cam/image_raw/metadata"
    done)
```

raw Bayer까지 남기려면 `/<camera>/image_raw`도 포함한다.

```bash
mkdir -p bags

ros2 bag record \
  -o bags/flir_multicam_raw_rgb_$(date +%Y%m%d_%H%M%S) \
  $(for cam in "${CAMERAS[@]}"; do
      echo "/$cam/image_raw /$cam/image_rgb/compressed /$cam/camera_info /$cam/image_raw/metadata"
    done)
```

카메라만 기록할 때는 `scripts/camera_bagging.sh`, 라이다까지 같이 기록할 때는
`scripts/all_sensors_bagging.sh`를 쓴다. 후자는 카메라 토픽 + `/ouster/lidar_packets`,
`/ouster/imu_packets`, `/ouster/metadata`, `/ouster/imu`, `/tf`, `/tf_static`을
하나의 mcap bag에 담고, 기록 전에 토픽이 실제로 올라와 있는지 확인한다.

```bash
scripts/all_sensors_bagging.sh
```

`/ouster/points`가 아니라 `lidar_packets`를 기록하는 이유는 points가 replay 때
패킷에서 복원되기 때문이다. `/ouster/metadata`가 없으면 복원이 불가능하므로
반드시 같이 담아야 한다.

## nuScenes Export

기본 변환은 최신 `bags/<bag_name>`을 찾아 `nuscenes_export/<bag_name>`에 쓴다.
bag에 `/<camera>/image_rgb/undistorted/compressed`가 있으면 우선 사용하고, 없으면
`/<camera>/image_rgb/compressed`를 `CameraInfo`로 왜곡 보정해서 저장한다.
namespace의 `camera_` 접두어는 채널명에서 빠진다: `camera_front_right` →
`CAM_FRONT_RIGHT`.

```bash
python3 scripts/bag_to_nuscenes.py --overwrite
```

raw/RGB를 같이 export:

```bash
python3 scripts/bag_to_nuscenes.py --raw-and-rgb --overwrite
```

nuScenes 표준 채널명으로 매핑:

```bash
python3 scripts/bag_to_nuscenes.py \
  --camera-channel camera_center=CAM_FRONT \
  --camera-channel camera_rear=CAM_BACK \
  --overwrite
```

출력 구조:

```text
nuscenes_export/<bag_name>/
  samples/CAM_CENTER/*.jpg
  samples/CAM_FRONT_RIGHT/*.jpg
  samples/CAM_FRONT_LEFT/*.jpg
  samples/CAM_SIDE_RIGHT_1/*.jpg
  ...
  v1.0-mini/*.json
  conversion_report.json
```

변환기는 bag의 `/cameraN/camera_info`를 우선 사용하고, 없으면
`calibration/flir_camera_info.yaml`을 fallback으로 사용한다. Extrinsic은
`calibration/flir_camera_extrinsics.yaml`의 `rotation_xyzw`와
`translation_xyz_m`을 nuScenes `calibrated_sensor` 테이블에 넣는다. 원본 이미지를
그대로 내보내야 할 때만 `--no-undistort`를 붙인다.

로컬 viewer:

```bash
python3 scripts/serve_nuscenes_images.py
```

기본 URL은 `http://127.0.0.1:8000`이다. 특정 export를 보려면 dataset root를
인자로 넘긴다.

```bash
python3 scripts/serve_nuscenes_images.py nuscenes_export/flir_multicam_20260509_200435
```

## Runtime Control

멀티캠 전체에 같은 ROS parameter를 적용하려면:

```bash
python3 scripts/multicam_param_set.py camera.ExposureAuto Off
python3 scripts/multicam_param_set.py camera.GainAuto Off
```

실제 Spinnaker node map 파라미터와 trigger/PTP 관련 세부 항목은
[`src/flir_spinnaker_camera/README.md`](src/flir_spinnaker_camera/README.md)를
기준으로 보면 된다.

## Outputs

- `calibration/flir_camera_info.yaml`: intrinsic calibration 결과
- `calibration/flir_camera_extrinsics.yaml`: multicam extrinsic 결과
- `calibration/captures/`: calibration 캡처 이미지, gitignore 처리
- `bags/`: ROS 2 bag, gitignore 처리
- `nuscenes_export/`: nuScenes 변환 결과, gitignore 처리

## Quick Diagnosis

- 카메라 연결이나 토픽이 궁금하면 `flir_spinnaker_camera`
- 체스보드 캡처와 intrinsic calibration이면 `flir_camera_calibration`
- calibration 결과를 적용한 영상을 쓰려면 `flir_camera_undistort_viewer`
- bag을 dataset으로 바꾸려면 `scripts/bag_to_nuscenes.py`
