# flir_camera_calibration

체스보드 기반 캘리브레이션을 위한 보조 패키지다.

## 역할

- `/image_rgb/compressed`를 받아 체스보드 검출
- 검출된 보드를 preview로 보여주고 annotated compressed 토픽 퍼블리시
- 샘플을 모아 calibration YAML 저장

## 입력과 출력

입력:

- `/image_rgb/compressed`

출력:

- `/calibration/image_annotated/compressed`
- `calibration/flir_camera_info.yaml`
- `calibration/captures/` 내부 캡처 이미지

## 조작 키

- `space`: 현재 보드 샘플 캡처
- `c`: `cv::calibrateCamera()` 실행 후 YAML 저장
- `r`: 샘플 초기화
- `q` 또는 `Esc`: 종료

## 동작 방식

- preview는 입력 프레임이 들어오는대로 바로 갱신된다.
- live preview 검출은 성능 때문에 downscale + FAST_CHECK를 쓴다.
- 실제 캡처 시에는 원본 full-resolution에서 다시 정밀 검출한다.
- 기본 preview 검출 폭은 `640px`이다.

## 보드 종류 (chessboard / ChArUco)

`board_type` 파라미터로 보드 종류를 고른다. intrinsic·extrinsic 노드 모두 지원한다.

- `board_type: "chessboard"` (기본): 순수 흑백 체스보드. `board_cols`, `board_rows`
  (inner corners), `square_size_m`를 사용한다.
- `board_type: "charuco"`: ChArUco 보드. `charuco_squares_x`, `charuco_squares_y`
  (전체 square 개수), `charuco_square_length_m`, `charuco_marker_length_m`,
  `aruco_dictionary`(예: `DICT_5X5_1000`)를 사용한다. `charuco_*`/`aruco_dictionary`
  값은 실제 출력한 보드와 정확히 일치해야 한다.

ChArUco는 마커 ID로 각 코너의 원점·방향이 유일하게 결정되므로, 순수 체스보드의
180도 방향 모호성이 없다. 멀티캠 extrinsic처럼 시점 차가 큰 경우 더 견고하다.

ChArUco로 실행:

```bash
# 단일 intrinsic
ros2 launch flir_camera_calibration calibration.launch.py \
  board_type:=charuco \
  charuco_squares_x:=8 charuco_squares_y:=7 \
  charuco_square_length_m:=0.12 charuco_marker_length_m:=0.09 \
  aruco_dictionary:=DICT_5X5_1000

# 카메라별 intrinsic
ros2 launch flir_camera_calibration multicam_calibration.launch.py \
  camera_name:=camera_center board_type:=charuco

# 멀티캠 extrinsic
ros2 launch flir_camera_calibration multicam_extrinsic_calibration.launch.py \
  board_type:=charuco
```

기본값을 매번 넘기기 번거로우면 `config/calibration.yaml`,
`config/extrinsic_calibration.yaml`의 `board_type`과 `charuco_*`를 바꿔 두면 된다.

## 자동 캡처 (auto-capture, intrinsic 전용)

혼자 하거나 bag을 재생하며 캘리브레이션할 때, `space`를 매번 누르지 않고 자동으로
캡처하게 할 수 있다. PC가 차 안에 있어 보드와 키보드를 동시에 다루기 어려운 상황에 유용하다.

- `auto_capture: true` 면 프레임이 들어오는 대로, **이미 캡처한 것과 충분히 다른 포즈**
  (화면 위치·겉보기 크기 기준)만 자동으로 캡처한다. 같은 포즈나 bag 반복 재생 시의 중복은
  건너뛴다.
- **품질 모드(기본)**: `auto_capture_target_rms > 0` 이면, 캡처를 모으며 매번 재캘리브해서
  **RMS가 목표 이하로 떨어지는 순간 저장하고 멈춘다.** 프레임 수를 장면에 맞춰 자동 조절하므로
  커버리지가 나쁘면 알아서 더 모은다. 목표에 못 닿으면 `auto_capture_max_frames`에서 최선값을
  저장하고 종료한다(항상 끝남).
- 카운트 모드: `auto_capture_target_rms: 0` 으로 두면 `auto_capture_target_frames` 장에서
  한 번 캘리브하고 끝낸다.
- `display_window: false` 로 두면 창 없이 **완전 헤드리스**로 돈다.

파라미터:

- `auto_capture` (기본 `false`)
- `auto_capture_target_rms` (기본 `1.5`, `0`이면 카운트 모드) — 이 값 이하 RMS면 저장+종료
- `auto_capture_max_frames` (기본 `50`) — 목표 못 닿아도 여기서 최선값 저장하고 종료
- `auto_capture_target_frames` (기본 `25`, 카운트 모드에서만 사용)
- `auto_capture_min_move_frac` (기본 `0.05`, preview 폭 대비 최소 포즈 차이)
- `auto_capture_min_interval_sec` (기본 `0.3`, 캡처 간 최소 간격 초)

**실시간(라이브) + 품질 자동 종료** — bag 없이, 키 없이, RMS 좋아지면 자동 종료:

```bash
# 터미널 A: 카메라 노드
ros2 launch flir_spinnaker_camera multicam.launch.py

# 터미널 B: 보드를 계속 흔들기만 하면 RMS<=1.5에서 저장+종료
ros2 launch flir_camera_calibration multicam_calibration.launch.py \
  camera_name:=camera_center board_type:=charuco auto_capture:=true
```

bag 재생으로 할 때도 동일하게 `board_type:=charuco auto_capture:=true`만 붙이면 된다.

품질은 커버리지가 좌우하니 화면 **구석·기울임·원근**을 골고루 담아야 RMS가 빨리 내려간다.
RMS가 목표에 잘 안 닿으면 커버리지를 더 채우거나 `auto_capture_target_rms:=2.0`으로 완화한다.

## 실행

```bash
source scripts/setup_flir_env.bash
ros2 launch flir_camera_calibration calibration.launch.py
```

체스보드 크기 override 예:

```bash
ros2 launch flir_camera_calibration calibration.launch.py \
  board_cols:=6 \
  board_rows:=5 \
  square_size_m:=0.08
```

## 결과 활용

저장된 `calibration/flir_camera_info.yaml`은 `flir_spinnaker_camera`의 `camera_info.yaml_path`에 바로 연결할 수 있다.

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py \
  camera_info_yaml_path:=calibration/flir_camera_info.yaml
```

## 멀티캠 intrinsic calibration

멀티캠에서는 `src/flir_spinnaker_camera/config/multicam_cameras.yaml`의
`camera_center`, `camera_front_right` 같은 inventory 이름을 기준으로 대상 카메라를 고른다.

```bash
ros2 launch flir_camera_calibration multicam_calibration.launch.py camera_name:=camera_front_left
```

저장 결과는 `calibration/flir_camera_info.yaml`의 `camera_info_by_serial` 아래에
카메라 serial별 entry로 upsert된다. 캡처 이미지는 기본적으로
`calibration/captures/<serial>/` 아래에 저장된다.

## 멀티캠 extrinsic calibration

여러 카메라가 같은 체커보드를 overlap region에서 볼 수 있게 둔 뒤 실행한다.

```bash
ros2 launch flir_camera_calibration multicam_extrinsic_calibration.launch.py
```

노드는 각 `/<camera>/image_rgb/compressed`와 `/<camera>/camera_info`를 구독한다.
기본 모드에서는 보드를 현재 같이 보고 있는 카메라가 2대 이상이면 observation을
수동 수집하고, 이렇게 모은 pairwise graph를 `reference_camera` 기준 rig frame으로
합성한다.

- `space`: 현재 observation 수동 캡처
- `c`: `calibration/flir_camera_extrinsics.yaml` 저장
- `r`: observation 초기화
- `q` 또는 `Esc`: 종료

기본 기준은 `camera_center`이며, 저장된 transform은 `flir_rig_frame` 기준
`<camera>_optical_frame` pose로 기록된다.
예를 들어 `camera_center-camera_front_right` overlap에서 `space`를 `min_observations`
이상 누르고, 이어서 `camera_front_right-camera_side_right_1` overlap에서도 같은 만큼
캡처하면 `camera_side_right_1` pose는
`camera_center -> camera_front_right -> camera_side_right_1` graph path로 합성된다. 연결되지 않은 카메라가 있거나
graph path의 가장 약한 edge observation 수가 `min_observations`보다 적으면 저장을
거부한다. 모든 카메라가 동시에 보드를 볼 때만 캡처하려면
`require_all_cameras_for_capture:=true`를 넘긴다.

## 메모

- `board_cols`, `board_rows`는 square 개수가 아니라 `inner corners` 기준이다.
- `sample_image_dir`가 비어 있지 않으면 캡처 원본도 저장된다.
- 기본 `calibration/captures/` 이미지는 gitignore 처리되어 있다.
