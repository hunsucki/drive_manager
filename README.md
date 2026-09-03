# drive_manager

헤드리스 Nav2 실행과 로봇 주행 미션 노드를 위한 ROS 2 패키지입니다.

## Update 0903 — 전방 라이다 필터와 map_0903 적용

2026-09-03 작업에서는 전방 상판 라이다의 뒤쪽을 가리는 고정 구조물 때문에 Nav2가
로봇 자신을 장애물로 판정하던 문제를 분석하고, 로봇 전방 기준 `-120° ~ +120°`만
사용하는 LaserScan 전처리를 추가했습니다. 동일한 필터 결과를 Cartographer 지도 작성과
AMCL/Nav2 주행에서 사용하고, 새로 작성한 `/root/map_0903.yaml`을 launch 인자로 선택할
수 있도록 구성했습니다.

### 변경 배경과 확인된 원인

기존 raw `/scan`을 `base_footprint`로 변환해 확인했을 때 padded robot footprint
내부에 37개의 측정점이 있었고, 거리는 약 `0.063 ~ 0.208 m`였습니다. Collision
Monitor의 `FootprintApproach.min_points`는 6이므로 이 자기반사만으로도 Nav2의 유효한
속도 명령이 `/cmd_vel`에서 차단될 수 있었습니다. 실제 증상은 다음과 같았습니다.

```text
START_ESCAPE에서는 잠깐 전진
-> AMCL/Nav2 활성화
-> controller와 velocity_smoother는 속도를 생성
-> collision_monitor가 구조물 반사를 충돌로 판정
-> 최종 /cmd_vel이 차단되고 progress checker가 실패
```

실행 중 스캔을 로봇 좌표 기준 30° 구간으로 분석한 결과, 30 cm 이내의 근거리 반사는
주로 후방 `+120° ~ +180°`와 `-180° ~ -150°`에 집중됐고 전방
`-120° ~ +120°`에는 나타나지 않았습니다. 이에 따라 현재 1차 적용 범위는 전방
240°를 남기고 후방 120°를 제거하는 것으로 결정했습니다.

### C1 스캔 각도와 로봇 전방의 관계

물리적으로 RPLIDAR C1의 `+X` 방향은 로봇의 `+X` 전방과 같은 방향으로 장착돼
있습니다. 다만 SLLIDAR 드라이버의 LaserScan 각도 변환 때문에 물리적 전방은 raw
`/scan`에서 `+/-180°`에 나타납니다. URDF의 `base_link -> base_scan` yaw `pi`는 이
드라이버 규칙을 보정하므로 제거하거나 0으로 바꾸면 안 됩니다.

현재 변환 관계는 다음과 같습니다.

```text
물리적 라이다 전방(raw 0°)
-> SLLIDAR /scan의 +/-180°
-> base_scan yaw pi 적용
-> 로봇 +X 전방 0°
```

따라서 로봇 전방 `-120° ~ +120°`를 남기기 위해 필터 설정은 raw LaserScan 기준
중심 `180°`, 반각 `120°`를 사용합니다. 결과적으로 raw 스캔 중앙
`-60° ~ +60°`가 마스킹되며, 이 영역이 로봇 후방 120°에 해당합니다.

### 추가된 front_scan_filter

`drive_manager/front_scan_filter.py`에 `sensor_msgs/msg/LaserScan` 필터 노드를
추가했습니다.

```text
/scan (raw, base_scan)
-> front_scan_filter
-> /scan_front_filtered (base_scan)
```

필터의 주요 동작은 다음과 같습니다.

- 입력과 출력 토픽 및 필터 중심/반각을 ROS 파라미터로 설정합니다.
- 각도 wrap-around를 고려해 `+180°`와 `-180°` 양쪽의 전방 측정값을 보존합니다.
- 제외할 빔은 배열에서 삭제하지 않고 range를 `NaN`으로 바꿉니다.
- 원본 `header.stamp`, `frame_id`, angle/range metadata와 intensity를 보존합니다.
- 입출력에는 sensor-data QoS를 사용합니다.
- raw `/scan`은 유지하므로 RViz 비교와 문제 분석에 계속 사용할 수 있습니다.

설정 파일은 `param/scan_filter.yaml`입니다.

```yaml
front_scan_filter:
  ros__parameters:
    input_topic: /scan
    output_topic: /scan_front_filtered
    front_center_angle_deg: 180.0
    front_half_angle_deg: 120.0
```

필터 실행 파일은 `setup.py`에 다음 이름으로 등록돼 있습니다.

```bash
ros2 run drive_manager front_scan_filter \
  --ros-args --params-file \
  /root/colcon_ws/install/drive_manager/share/drive_manager/param/scan_filter.yaml
```

일반 운용에서는 직접 실행할 필요가 없습니다. `nav2_headless.launch.py`와 STELLA의
`stella_cartographer/launch/cartographer.launch.py`가 필터를 자동 실행합니다.

### Nav2 센서 입력 변경

`param/stella.yaml`에서 다음 구성요소의 입력을 raw `/scan`에서
`/scan_front_filtered`로 변경했습니다.

| 구성요소 | 0903 이후 입력 | 역할 |
| --- | --- | --- |
| AMCL | `/scan_front_filtered` | map 기준 localization |
| local costmap obstacle layer | `/scan_front_filtered` | 근거리 장애물 marking/clearing |
| global costmap obstacle layer | `/scan_front_filtered` | 전역 장애물 marking/clearing |
| Collision Monitor | `/scan_front_filtered` | 최종 속도 충돌 검사 |
| mission_driver readiness | `/scan_front_filtered` | 스캔 freshness와 TF 준비 확인 |

기존 costmap에는 raw `/scan`과 publisher가 없는 `/scan_filtered`를 동시에 등록한
`scan scan_2` 구성이 있었습니다. 현재 전방 라이다 한 대만 검증하는 단계이므로
`scan_2`를 제거하고 필터된 한 개의 observation source만 사용합니다.

`nav2_headless.launch.py`는 Nav2와 함께 `front_scan_filter`를 실행하며, 필터가
비정상 종료되면 2초 뒤 respawn합니다. 필터 파라미터와 `use_sim_time`도 launch에서
함께 전달합니다.

### Cartographer 지도 작성 변경

`STELLA_N5_REMOTEPC_ROS2/stella_cartographer`에도 같은 필터를 연결했습니다.

- `stella_cartographer/package.xml`에 `drive_manager` 실행 의존성을 추가했습니다.
- `cartographer.launch.py`가 `front_scan_filter`를 자동 실행합니다.
- Cartographer의 `scan` 입력을 `/scan_front_filtered`로 remap합니다.
- `use_sim_time`을 Cartographer, occupancy grid, RViz, scan filter에 동일하게 전달합니다.

이 구성으로 맵 작성과 AMCL 주행이 동일한 라이다 관측 범위를 사용합니다. 로봇에
고정된 상판 구조물은 환경 지도로 들어가지 않으며, 주행 시에도 장애물로 재등장하지
않습니다.

rosbag으로 맵을 작성할 때 실기 로봇 데이터와 섞이면 과거 bag timestamp와 현재 TF가
충돌해 `TF_OLD_DATA` 및 `Message Filter dropping message`가 발생합니다. 가장 안전한
방법은 별도 ROS domain과 simulated clock을 사용하는 것입니다.

터미널 1:

```bash
cd /root/colcon_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch stella_cartographer cartographer.launch.py use_sim_time:=true
```

터미널 2:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42

ros2 bag play /path/to/bag --clock
```

bag에는 최소 `/scan`, `/odom`, `/tf`, `/tf_static`이 있어야 합니다. bag에
`/tf_static`이 없다면 같은 domain에서 올바른 URDF의 `robot_state_publisher`를 별도로
실행해야 합니다. 실기 데이터로 맵을 작성할 때는 `use_sim_time:=false`를 사용하고
rosbag player를 동시에 실행하지 않습니다.

맵 저장 예시:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source /root/colcon_ws/install/setup.bash

ros2 run nav2_map_server map_saver_cli -f /root/map_0903
```

### map_0903과 맵 선택 방식

0903 필터를 적용해 생성한 파일은 다음과 같습니다. 이 파일들은 패키지 내부가 아니라
`/root`에 저장돼 있습니다.

```text
/root/map_0903.yaml
/root/map_0903.pgm
```

확인된 속성은 다음과 같습니다.

- resolution: `0.05 m/cell`
- image size: `524 x 303`
- origin: `[-11.146, -5.182, 0]`
- Cartographer scan rate: 약 `10.01 Hz`
- Cartographer odom rate: 약 `2.5 Hz`
- 생성된 submap: 3개
- 확인된 scan-match score: 주로 약 `74 ~ 84%`
- 필터 출력의 유효 로봇 각도: 약 `-119.2° ~ +119.7°`
- 필터 후 `+/-120°` 밖 유효점: 0개
- padded robot footprint 내부 유효점: 기존 37개에서 0개로 감소

기존 `map_0829`와 월드 좌표계의 occupied 영역 경계를 비교했을 때 차이는 대체로
수 cm에서 약 10 cm 수준이었으며, 큰 지도 왜곡이나 필터로 인한 벽 손실은 확인되지
않았습니다.

`drive_manager.launch.py`는 `map` launch argument를 제공하며 현재 기본값은
`/root/map_0903.yaml`입니다. 다른 맵을 사용할 때 소스를 다시 수정할 필요 없이 다음처럼
지정합니다.

```bash
export ROS_DOMAIN_ID=0
ros2 launch drive_manager drive_manager.launch.py map:=/root/map_0903.yaml
```

`nav2_headless.launch.py`를 직접 실행하면 해당 파일의 기본 map 설정이 사용되므로,
운영 시에는 가급적 최상위 `drive_manager.launch.py`에 `map:=...`을 전달합니다.

### START 동작 검증

웹 앱에서 START를 누른 실기 시험에서 다음 흐름을 확인했습니다.

```text
시간 기반 START_ESCAPE 전진
-> localization reset/start
-> departure_initial_pose 반복 발행
-> 최신 AMCL pose 3회 연속 및 map TF 검사 통과
-> navigation 활성화와 costmap clear
-> 첫 patrol goal 주행 시작
```

현재 구현상 START_ESCAPE 이후 Nav2 제어로 로봇이 다시 움직였다면 AMCL 안정 조건을
통과하고 목표 주행 단계로 진입한 것입니다. 최종 판단 시에는 RViz에서 필터된 스캔이
지도 벽과 겹치는지, 로봇이 첫 patrol point 방향의 global plan을 따라가는지도 함께
확인합니다.

### 웹 FORCE 복구 동작 보완

동일 작업에서 mission/drive/Nav2가 `ERROR ...` 상태를 발행하더라도 FORCE 모드를
자동으로 해제하지 않도록 변경했습니다. `ERROR manual_nav2_not_ready`처럼 Nav2가
정상 주행을 시작하지 못한 상태에서도 운영자가 카메라를 확인하고 저속 FORCE로 안전한
위치까지 복구할 수 있습니다.

다음 상태는 기존대로 FORCE를 차단하거나 해제합니다.

- 실제 `DOCKING` 진행 중
- ESTOP latch 상태
- START_ESCAPE 등 다른 노드가 `/cmd_vel`을 직접 소유하는 단계

FORCE는 Collision Monitor를 우회하므로 일반 주행 기능이 아니라 제한된 복구
수단으로만 사용해야 합니다.

### 현재 범위와 후속 작업

이번 0903 구성은 전방 라이다 한 대만 사용하는 1차 검증 구성입니다.

- 후방 120°는 costmap과 Collision Monitor에서 보이지 않습니다.
- 회전과 전진 주행은 가능하지만 후진/backup 동작의 후방 충돌 보호는 제한됩니다.
- START_ESCAPE는 Nav2를 pause하고 `/cmd_vel`에 직접 속도를 발행하므로 Collision
  Monitor를 우회합니다. 도크 이탈 중 전방 안전 감시는 추후 별도 보완이 필요합니다.
- `base_scan2` 프레임과 두 번째 라이다 위치는 URDF에 선언돼 있지만, 현재
  `/scan_front_filtered` 파이프라인에는 두 번째 라이다를 연결하지 않았습니다.
- 후속 단계에서는 앞 라이다의 전방 영역과 뒤 라이다의 후방 영역을 각각 필터링해
  Cartographer, AMCL, costmap, Collision Monitor에 함께 제공할 예정입니다.

`STELLA_N5_REMOTEPC_ROS2` 작업 트리에는 두 번째 라이다 이동을 반영하기 위해 세 URDF의
`base_scan2` 위치를 `xyz="-0.166 0.0 0.223"`, yaw를 `3.1415`로 변경한 내용도
포함돼 있습니다. 실제 설치 위치의 `x/y/z`를 다시 측정한 뒤 확정해야 합니다.
`stella_navigation2/param/stella_senser.yaml`의 카메라 프로필은 `use_realsense`로
변경돼 있습니다. 이 두 변경은 현재 단일 전방 라이다 필터 동작과는 별도입니다.

### 빌드 및 검증

두 패키지를 함께 빌드합니다.

```bash
cd /root/colcon_ws
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select drive_manager stella_cartographer \
  --symlink-install

source install/setup.bash
```

검증 시 다음 결과를 확인했습니다.

- `drive_manager`, `stella_cartographer` 빌드 성공
- 두 launch 파일의 `--show-args` 로딩 성공
- `drive_manager` 전체 Python test 23개 통과
- 신규 각도 wrap/boundary 필터 test 3개 포함
- `git diff --check` 통과

필터 출력 확인:

```bash
ros2 topic info /scan_front_filtered -v
ros2 topic echo /scan_front_filtered --qos-reliability best_effort
```

빌드 중 `setup.py: option --editable/--uninstall not recognized`가 발생했던 환경에서는
시스템의 `setuptools 81.0.0`과 ROS 2 Python 빌드 도구의 호환 문제가 있었고,
`setuptools 79.0.1`로 맞춘 뒤 빌드가 정상화됐습니다. 기존 빌드 실패가 남긴 install
symlink와 충돌하면 해당 패키지의 `build/drive_manager` 및 `install/drive_manager`만
정리한 뒤 다시 빌드해야 하며, 작업공간 전체를 무조건 삭제하지 마십시오.

## 통합 서버 실행

```bash
source /root/colcon_ws/install/setup.bash
ros2 launch drive_manager drive_manager.launch.py
```

이 명령 하나가 rosbridge, command/mission manager, `nav2_supervisor`, Nav2를 함께
실행합니다. Nav2 프로세스는 떠 있지만 lifecycle은 `unconfigured` 상태로 대기하며,
START/HOME 명령이 있을 때만 supervisor가 활성화합니다. 별도의
`nav2_headless.launch.py`를 동시에 실행하면 노드가 중복되므로 실행하지 마십시오.

Nav2만 수동 점검하려면 다음처럼 실행할 수 있습니다.

```bash
source /root/colcon_ws/install/setup.bash
ros2 launch drive_manager nav2_headless.launch.py use_rviz:=true autostart:=true
```

이 launch 파일은 패키지 안의 URDF를 사용해 `robot_state_publisher`를 실행하고,
`drive_manager`에 포함된 STELLA Nav2 파라미터로 `nav2_bringup`을 함께 실행합니다.

## Two Point Mission 실행

```bash
source /root/colcon_ws/install/setup.bash
ros2 run drive_manager two_point
```

## Mobile Mission 실행

`drive_manager.launch.py`는 모바일 앱과 주행에 필요한 프로세스를 함께 실행합니다.

- `rosbridge_websocket`: 모바일 앱 WebSocket 연결용입니다. 기본 포트는 `9090`입니다.
- `command_manager`: 앱 명령 `/robot_command`를 받아 내부 명령 `/mission_command`로 전달합니다.
- `mission_driver`: `param/mission_config.yaml`을 읽고 Nav2 `/navigate_to_pose` 액션으로 실제 주행을 수행합니다.
- `nav2_supervisor`: localization/navigation lifecycle을 순서대로 시작·정지·복구합니다.
- `web_teleop`: 웹 수동 조종을 safe/force 두 경로로 중계하고 watchdog과 Nav2 상호잠금을 적용합니다.
- `Nav2`: 기본적으로 lifecycle-inactive 상태로 대기합니다.

```bash
source /root/colcon_ws/install/setup.bash
ros2 launch drive_manager drive_manager.launch.py
```

## 앱 명령

명령 예시:

```bash
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: START}"
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: HOME}"
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: STOP}"
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: ESTOP}"
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: RESET}"
```

rosbridge WebSocket에서 직접 publish할 때는 아래 JSON을 보냅니다.

```json
{
  "op": "publish",
  "topic": "/robot_command",
  "msg": {
    "data": "START"
  }
}
```

상태 확인:

```bash
ros2 topic echo /robot_status
```

앱에서 상태를 구독할 때는 `/robot_status`를 subscribe합니다.

```json
{
  "op": "subscribe",
  "topic": "/robot_status",
  "type": "std_msgs/String"
}
```

미션 좌표 확인:

```bash
ros2 topic echo /mission_route_points
```

웹 앱에서는 `/mission_route_points`를 `std_msgs/String`으로 구독한 뒤 `msg.data`를 JSON으로 파싱하면 됩니다.

```json
{
  "op": "subscribe",
  "topic": "/mission_route_points",
  "type": "std_msgs/String"
}
```

발행 데이터에는 `home_to_patrol_pose`, `home_to_dock_pose`, `patrol_points`, 그리고 실제 주행 순서와 자동 계산된 patrol yaw를 담은 `navigation_sequence`가 포함됩니다.

웹 지도에서 로봇 위치는 `/amcl_pose` 대신 `/robot_pose`를 구독하는 것을 권장합니다.
주행 중에는 AMCL 위치를 전달하고, Nav2가 꺼진 도킹 상태에서는 설정된
`docked_pose`를 transient-local로 계속 제공합니다. 위치 출처는
`/robot_pose_status`의 `AMCL`, `DOCKED`, `DOCKED_ASSUMED`으로 구분할 수 있습니다.

명령 동작:

- `START`: Nav2가 비활성인 상태로 dock escape를 실행하고, `departure_initial_pose`로 AMCL을 초기화한 뒤 patrol_points를 순회합니다. 복귀·도킹 성공 후 Nav2를 다시 reset합니다.
- `HOME`: 어디에 있든 현재 미션을 중단하고 home_to_dock_pose로 이동한 뒤 SSH 도킹 명령을 실행합니다.
- `STOP`: 현재 Nav2 goal 또는 도킹 SSH 프로세스를 중단하고 `/cmd_vel` 0을 발행합니다.
- `ESTOP`: STOP과 같지만 latch 상태가 되어 `RESET` 전까지 START/HOME을 거부합니다.
- `RESET`: ESTOP latch를 해제합니다.

START 중 START를 다시 누르면 `mission_driver`가 `BUSY` 상태를 발행하고 기존 미션을 계속 진행합니다. START 중 HOME을 누르면 현재 goal을 취소하고 HOME 복귀 미션으로 전환합니다.

## 웹 앱 연동 가이드

웹 앱은 `ws://<서버 IP>:9090`의 rosbridge WebSocket에 연결합니다. 이번 변경으로
기존 `/robot_command`와 지도 표시 외에 다음 기능을 구현해야 합니다.

1. safe/force 수동 조종 publisher 두 개
2. 수동 조종 서버 상태 subscriber 두 개
3. 버튼을 누르는 동안만 동작하는 10~20 Hz 송신 루프
4. 수동 이동 후 `/initialpose`를 발행하는 2D Pose Estimate
5. `/web_teleop/active=false`를 확인한 뒤 HOME을 보내는 복구 흐름

### 웹 앱이 사용하는 ROS 인터페이스

| 방향 | 토픽 | 타입 | 웹 앱에서의 역할 |
| --- | --- | --- | --- |
| Web → ROS | `/robot_command` | `std_msgs/msg/String` | `START`, `HOME`, `STOP`, `ESTOP`, `RESET` |
| Web → ROS | `/web_teleop/mode_request` | `std_msgs/msg/String` | FORCE 래치 설정(`FORCE`) 또는 해제(`SAFE`) |
| Web → ROS | `/cmd_vel_web_safe` | `geometry_msgs/msg/Twist` | 충돌 보호를 사용하는 기본 수동 조종 |
| Web → ROS | `/cmd_vel_web_force` | `geometry_msgs/msg/Twist` | 충돌 보호를 우회하는 저속 탈출 조종 |
| Web → ROS | `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | AMCL 위치 재설정 |
| ROS → Web | `/web_teleop/status` | `std_msgs/msg/String` | 수동 조종 전환 및 오류 상태 |
| ROS → Web | `/web_teleop/active` | `std_msgs/msg/Bool` | 수동 조종/전환 중인지 판정 |
| ROS → Web | `/robot_status` | `std_msgs/msg/String` | 미션 상태 및 명령 결과 |
| ROS → Web | `/robot_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 지도에 표시할 현재 위치 |
| ROS → Web | `/robot_pose_status` | `std_msgs/msg/String` | 위치 출처: `AMCL`, `DOCKED`, `DOCKED_ASSUMED` |
| ROS → Web | `/mission_route_points` | `std_msgs/msg/String` | HOME/순회 좌표와 주행 순서 JSON |

지도에는 `/amcl_pose` 대신 `/robot_pose`를 표시해야 합니다. `/robot_pose`는 주행
중에는 AMCL 위치를 전달하고, Nav2가 reset된 도킹 상태에서는 설정된 `docked_pose`를
transient-local 메시지로 제공합니다.

### safe와 force의 차이

| 입력 토픽 | 서버 속도 제한 | 실제 출력 경로 | 사용 조건 |
| --- | --- | --- | --- |
| `/cmd_vel_web_safe` | 0.15 m/s, 0.40 rad/s | `/cmd_vel_nav` → velocity smoother → collision monitor → `/cmd_vel` | 모든 일반 수동 조종에서 먼저 사용 |
| `/cmd_vel_web_force` | 0.08 m/s, 0.25 rad/s | Nav2 goal 취소 → navigation PAUSE 확인 → `/cmd_vel` | safe가 collision monitor에 막혔을 때만 사용 |

`safe`는 기존 collision monitor를 통과하므로 장애물이나 잘못 남은 costmap 때문에
움직이지 않을 수 있습니다. `force`는 이 보호를 의도적으로 우회합니다. 서버가
navigation lifecycle의 PAUSE를 확인하지 못하면 force 속도를 출력하지 않습니다.
`ERROR manual_nav2_not_ready`를 포함한 mission/drive/Nav2 오류 상태는 복구를 위해
force 진입을 막지 않습니다. 다만 E-stop이 래치된 상태, 실제 도킹 진행 중, 또는
다른 직접 속도 출력 단계에서는 force를 거부합니다.

force UI는 명시적인 토글 버튼으로 구현합니다. 활성화할 때 `FORCE`, 비활성화할 때
`SAFE`를 `/web_teleop/mode_request`에 한 번 발행합니다. 조이스틱을 놓거나 0 Twist를
보내는 것은 로봇만 정지시키며 FORCE ARMED 상태는 해제하지 않습니다. 카메라 화면이
보이는 상태에서만 활성화하고, 사람·계단·낙하 위험이 있는 장소에서는 사용하지 마십시오.

서버에는 다음 우선순위와 제한이 적용됩니다.

- FORCE ARMED에서는 `/cmd_vel_web_force`만 허용합니다.
- SAFE 전환이 완료된 상태에서는 `/cmd_vel_web_safe`만 허용합니다.
- `linear.x`와 `angular.z`만 사용하며 나머지 축은 서버가 0으로 만듭니다.
- `linear.x > 0`은 전진, `< 0`은 후진입니다.
- `angular.z > 0`은 좌회전, `< 0`은 우회전입니다.
- NaN, Infinity와 제한을 넘는 값은 서버에서 차단하거나 clamp합니다.
- 마지막 명령 이후 0.35초가 지나면 서버가 0 속도를 발행합니다. FORCE ARMED 상태는
  그대로 유지되며 다음 FORCE Twist를 받을 수 있습니다.

### rosbridge 연결 및 구독

연결 직후 필요한 토픽을 advertise/subscribe합니다. 연결이 끊겼다가 다시 연결되면
동일한 초기화를 다시 수행해야 합니다.

```javascript
// 웹 앱과 drive_manager가 같은 호스트라면 그대로 사용할 수 있습니다.
// 호스트가 다르면 배포 설정의 drive_manager IP로 교체합니다.
const ROSBRIDGE_URL = `ws://${window.location.hostname}:9090`;
const ros = new WebSocket(ROSBRIDGE_URL);

function sendRos(message) {
  if (ros.readyState === WebSocket.OPEN) {
    ros.send(JSON.stringify(message));
  }
}

ros.addEventListener("open", () => {
  for (const publisher of [
    {topic: "/web_teleop/mode_request", type: "std_msgs/msg/String"},
    {topic: "/cmd_vel_web_safe", type: "geometry_msgs/msg/Twist"},
    {topic: "/cmd_vel_web_force", type: "geometry_msgs/msg/Twist"},
    {topic: "/robot_command", type: "std_msgs/msg/String"},
    {
      topic: "/initialpose",
      type: "geometry_msgs/msg/PoseWithCovarianceStamped",
    },
  ]) {
    sendRos({
      op: "advertise",
      topic: publisher.topic,
      type: publisher.type,
    });
  }

  for (const topic of [
    "/web_teleop/status",
    "/web_teleop/active",
    "/robot_status",
    "/robot_pose",
    "/robot_pose_status",
    "/mission_route_points",
  ]) {
    sendRos({op: "subscribe", topic, throttle_rate: 100});
  }
});
```

실제 서버 주소는 배포 환경에 맞게 바꿉니다. TLS를 적용한 웹 페이지에서는 브라우저의
mixed-content 정책 때문에 `ws://`가 차단될 수 있으므로 reverse proxy를 통한
`wss://` 구성이 필요할 수 있습니다.

### 수동 조종 송신 루프

Twist를 한 번만 보내는 방식으로 구현하면 안 됩니다. 사용자가 버튼이나 조이스틱을
누르는 동안 10~20 Hz로 계속 보내야 하며, `pointerup`, `pointercancel`, 창 focus
상실과 WebSocket 종료 시 즉시 로컬 송신 루프를 중단해야 합니다.

```javascript
const TELEOP_PERIOD_MS = 1000 / 15;
const ZERO_TWIST = {
  linear: {x: 0.0, y: 0.0, z: 0.0},
  angular: {x: 0.0, y: 0.0, z: 0.0},
};

let teleopTimer = null;
let requestedMode = null;
let latestTwist = ZERO_TWIST;

function requestTeleopMode(mode) {
  sendRos({
    op: "publish",
    topic: "/web_teleop/mode_request",
    msg: {data: mode},
  });
}

function teleopTopic(mode) {
  return mode === "force"
    ? "/cmd_vel_web_force"
    : "/cmd_vel_web_safe";
}

function publishTwist(mode, twist) {
  sendRos({
    op: "publish",
    topic: teleopTopic(mode),
    msg: twist,
  });
}

function stopTeleop() {
  if (teleopTimer !== null) {
    clearInterval(teleopTimer);
    teleopTimer = null;
  }

  if (requestedMode !== null) {
    // 즉시 정지를 돕는 0 명령입니다. 서버 watchdog도 독립적으로 동작합니다.
    publishTwist(requestedMode, ZERO_TWIST);
  }

  requestedMode = null;
  latestTwist = ZERO_TWIST;
}

function startTeleop(mode, linearX, angularZ) {
  stopTeleop();
  requestedMode = mode;
  latestTwist = {
    linear: {x: linearX, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: angularZ},
  };

  publishTwist(requestedMode, latestTwist);
  teleopTimer = setInterval(() => {
    publishTwist(requestedMode, latestTwist);
  }, TELEOP_PERIOD_MS);
}

window.addEventListener("blur", stopTeleop);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) stopTeleop();
});
ros.addEventListener("close", stopTeleop);
```

`stopTeleop()`은 속도 송신만 멈춥니다. FORCE 토글을 끌 때는 별도로
`requestTeleopMode("SAFE")`를 호출해야 합니다.

방향 버튼 연결 예시는 다음과 같습니다.

```javascript
forwardButton.addEventListener("pointerdown", () => {
  startTeleop("safe", 0.10, 0.0);
});
leftButton.addEventListener("pointerdown", () => {
  startTeleop("safe", 0.0, 0.25);
});
forceBackwardButton.addEventListener("pointerdown", () => {
  startTeleop("force", -0.05, 0.0);
});

forceToggleButton.addEventListener("click", () => {
  if (teleopStatus === "FORCE") {
    stopTeleop();
    requestTeleopMode("SAFE");
  } else {
    requestTeleopMode("FORCE");
  }
});

for (const button of [forwardButton, leftButton, forceBackwardButton]) {
  button.addEventListener("pointerup", stopTeleop);
  button.addEventListener("pointercancel", stopTeleop);
  button.addEventListener("pointerleave", stopTeleop);
}
```

아날로그 조이스틱을 사용한다면 정규화된 입력 `[-1, 1]`에 웹 측 최대 속도를 곱해
`latestTwist`를 갱신합니다. 서버에도 최종 속도 제한이 있으므로 웹 제한을 잘못
설정하더라도 설정값 이상의 속도는 출력되지 않습니다.

### 상태 처리와 UI 규칙

rosbridge의 수신 메시지에서 `op === "publish"`와 `topic`을 확인해 상태를 저장합니다.

```javascript
let teleopStatus = "DISABLED";
let teleopActive = null; // 상태를 처음 받을 때까지 unknown
startButton.disabled = true;
homeButton.disabled = true;

ros.addEventListener("message", (event) => {
  const packet = JSON.parse(event.data);
  if (packet.op !== "publish") return;

  if (packet.topic === "/web_teleop/status") {
    teleopStatus = packet.msg.data;
    renderTeleopStatus(teleopStatus);

    if (teleopStatus.startsWith("ERROR")) {
      stopTeleop();
      showError(teleopStatus);
    }
  }

  if (packet.topic === "/web_teleop/active") {
    teleopActive = packet.msg.data;
    startButton.disabled = teleopActive;
    homeButton.disabled = teleopActive;
  }
});
```

상태의 의미는 다음과 같습니다.

| `/web_teleop/status` | UI 처리 |
| --- | --- |
| `DISABLED` | 수동 조종 해제. START/HOME을 허용할 수 있음 |
| `TRANSITIONING_SAFE` | localization/navigation 시작 및 goal 취소 중. 아직 움직임을 보장하지 않음 |
| `SAFE` | FORCE 해제 완료. safe 속도 전달 가능, 입력이 없으면 `active=false` |
| `TRANSITIONING_FORCE` | goal 취소 및 navigation PAUSE 중. 아직 force 출력 안 됨 |
| `FORCE` | FORCE ARMED. 입력/속도가 0이어도 `active=true`를 유지 |
| `ERROR ...` | 송신 중단, 원인 표시. 자동으로 force를 재시도하지 않음 |

START/HOME 버튼은 `/web_teleop/active=true`인 동안 비활성화해야 합니다. FORCE에서는
조이스틱을 놓아도 `active=true`이므로 FORCE 토글을 끄고 `SAFE/active=false`를 확인한
다음 START/HOME을 발행합니다.

```javascript
function publishRobotCommand(command) {
  if (
    (command === "START" || command === "HOME") &&
    teleopActive !== false
  ) {
    showError("수동 조종을 먼저 해제하세요.");
    return;
  }

  sendRos({
    op: "publish",
    topic: "/robot_command",
    msg: {data: command},
  });
}
```

서버도 같은 상호잠금을 적용하므로 active 상태에서 들어온 START/HOME은
`MANUAL_CONTROL_ACTIVE`로 거부됩니다.

### 2D Pose Estimate 구현

로봇을 수동으로 크게 이동했거나 지도상의 로봇 마커가 실제 위치와 다를 때만
`/initialpose`를 발행합니다. 수동 이동이 끝나기 전에 보내면 바로 위치가 다시
어긋날 수 있으므로 반드시 다음 순서를 지킵니다.

```text
safe 또는 force 수동 이동
-> 모든 방향 버튼 해제
-> FORCE를 사용했다면 /web_teleop/mode_request에 SAFE 발행
-> /web_teleop/active=false 확인
-> 지도에서 실제 위치와 전방 방향 선택
-> /initialpose 발행
-> 새로운 /robot_pose 수신 및 위치 확인
-> HOME 명령
```

지도에서 받은 `(x, y, yaw)`를 다음처럼 발행합니다. yaw 단위는 degree가 아니라
radian이며, 지도 좌표계 기준 회전입니다.

```javascript
function publishInitialPose(x, y, yaw) {
  if (teleopActive) {
    showError("로봇을 정지한 뒤 위치를 설정하세요.");
    return;
  }

  const nowMs = Date.now();
  const covariance = Array(36).fill(0.0);
  covariance[0] = 0.25 * 0.25;                  // x variance
  covariance[7] = 0.25 * 0.25;                  // y variance
  covariance[35] = 0.2618 * 0.2618;             // yaw variance (15 deg)

  sendRos({
    op: "publish",
    topic: "/initialpose",
    msg: {
      header: {
        stamp: {
          sec: Math.floor(nowMs / 1000),
          nanosec: (nowMs % 1000) * 1000000,
        },
        frame_id: "map",
      },
      pose: {
        pose: {
          position: {x, y, z: 0.0},
          orientation: {
            x: 0.0,
            y: 0.0,
            z: Math.sin(yaw / 2.0),
            w: Math.cos(yaw / 2.0),
          },
        },
        covariance,
      },
    },
  });
}
```

발행 직후 HOME을 자동 실행하지 말고, 발행 이후의 새로운 `/robot_pose`가 도착하고
지도 마커와 방향이 맞는지 운영자가 확인하도록 해야 합니다. HOME 처리 과정에서
navigation을 다시 활성화하고 costmap을 clear합니다.

도킹 완료 후에도 map_server와 AMCL localization은 복원됩니다. `/initialpose`를 발행하면
Nav2 navigation까지 준비되지만 goal은 자동 전송되지 않습니다. 이후 START를 누르면
수동 위치와 관계없이 기존 도크 탈출 및 자동 초기화 절차를 다시 수행합니다.

### 수동 복구 시나리오

순회 중 로봇이 장애물 영역이나 잘못된 costmap 때문에 멈춘 경우의 권장 흐름입니다.

```text
1. 카메라 스트림 확인
2. /cmd_vel_web_safe로 먼저 탈출 시도
3. safe가 collision monitor에 막힐 때만 FORCE 토글을 켜고 저속 이동
4. 안전한 위치에서 모든 수동 입력 해제
5. FORCE 토글을 끄고 `/web_teleop/mode_request`에 SAFE 발행
6. `/web_teleop/active=false` 확인
7. 실제 위치와 지도 위치가 다르면 2D Pose Estimate
8. 새로운 /robot_pose와 방향 확인
9. HOME을 눌러 도킹 스테이션으로 복귀
```

수동 모드에 진입하면 진행 중이던 `NavigateToPose` goal이 취소되고 기존 START 미션은
실패로 종료됩니다. 현재 `START`는 도킹 위치에서 출발하는 전용 시퀀스이므로 중간
위치에서 다시 누르면 안 됩니다. 수동 복구 뒤 HOME 복귀는 지원하지만, 중단된
waypoint부터 순회를 계속하려면 별도의 `RESUME` 명령과 진행상태 저장 기능이 필요합니다.

### 웹 앱 구현 체크리스트

- rosbridge 재연결 시 publisher advertise와 subscriber 등록을 다시 수행한다.
- safe를 기본값으로 사용하고 FORCE는 명시적 토글로만 활성화/해제한다.
- FORCE 조이스틱을 놓을 때는 0 Twist만 보내고, 토글을 끌 때만 SAFE를 요청한다.
- Twist를 10~20 Hz로 보내고 브라우저 blur/숨김/연결 종료 때 송신을 중단한다.
- `TRANSITIONING_*` 상태에서는 아직 로봇이 움직인다고 가정하지 않는다.
- `ERROR`를 받으면 송신 루프를 중단하고 사용자에게 원인을 표시한다.
- `/web_teleop/active=true`일 때 START/HOME 버튼을 비활성화한다.
- `/robot_pose`와 `/robot_pose_status`를 지도 위치의 단일 입력으로 사용한다.
- `/initialpose`는 로봇 정지와 `active=false`를 확인한 뒤에만 발행한다.
- 수동 복구 뒤에는 START가 아니라 HOME을 사용한다.

## Mission 설정

HOME 좌표와 START 순회 좌표는 `param/mission_config.yaml`에서 관리합니다.
HOME 계열 좌표는 방향이 중요해서 yaw를 직접 넣고, 순회 포인트는 `[x, y]`만 넣습니다.
순회 포인트의 yaw는 다음 목표 좌표를 바라보도록 `mission_driver`가 자동 계산합니다.

```yaml
command_manager:
  ros__parameters:
    command_topic: "/robot_command"
    status_topic: "/robot_status"
    mission_command_topic: "/mission_command"
    mission_status_topic: "/mission_status"

mission_driver:
  ros__parameters:
    route_points_topic: "/mission_route_points"
    route_points_publish_period_sec: 1.0

    home_to_patrol_pose: [-0.265, 4.405, -1.5708]
    home_to_dock_pose: [-0.265, 4.405, 1.0472]
    departure_initial_pose: [-0.265, 4.405, -1.5708]
    docked_pose: [-0.265, 4.405, 1.0472]
    navigate_to_home_to_patrol_pose: false
    reset_nav2_after_docking: true

    patrol_points: ["point_1", "point_2", "point_3"]
    patrol:
      point_1: [-0.165, -0.145]
      point_2: [3.735, -0.045]
      point_3: [-0.165, -0.145]

    start_escape_enabled: true
    start_escape_linear_x: 0.10
    start_escape_angular_z: 0.0
    start_escape_duration_sec: 2.0
    start_escape_stop_sec: 0.5

    docking_mode: "ssh"
    docking_ssh_user: "user"
    docking_ssh_host: "192.168.0.3"
    docking_ssh_port: 22
    docking_ssh_identity_file: "/root/.ssh/id_ed25519_drive_manager"
    docking_ssh_strict_host_key_checking: "accept-new"
    docking_remote_setup_files:
      - "/opt/ros/jazzy/setup.bash"
      - "/home/user/colcon_ws/install/setup.bash"
    docking_remote_command: "ros2 run docking dock_turn_backup"
    docking_timeout_sec: 120.0
    docking_stop_grace_sec: 3.0
```

START 미션 순서는 항상 아래와 같습니다.

```text
WAIT_ROBOT (/odom, /scan, TF 최신 상태 확인)
-> Nav2 navigation inactive
START_ESCAPE
-> localization STARTUP
-> departure_initial_pose 발행
-> AMCL 공분산 및 map → base_link TF 확인
-> navigation STARTUP 및 costmap clear
-> patrol_points 순서대로 순회
-> HOME_TO_DOCK
-> navigation PAUSE
-> SSH docking command
-> 도킹 성공(exit code 0)
-> localization/navigation RESET
-> /robot_pose에 docked_pose 유지
```

`start_escape_*`는 도킹스테이션에서 빠져나오기 위한 open-loop 동작입니다. 현재
설정은 `/cmd_vel`로 0.1 m/s 전진을 2초 수행합니다. 그 직후의 실제 지도 좌표와
전방 방향을 측정해 `departure_initial_pose`를 보정해야 합니다. 도킹 최종 체결
위치도 `home_to_dock_pose`와 다를 수 있으므로 `docked_pose`를 별도로 보정하십시오.

## SSH 도킹

라즈베리파이의 `docking` 패키지가 지켜야 할 종료 코드, 신호 처리, `/cmd_vel`
소유권, DDS/SSH 제약과 통합 시험 절차는
[원격 도킹 패키지 연동 규격](DOCKING_PACKAGE_INTEGRATION.md)을 참고하십시오.

도킹 단계에서는 `mission_driver`가 라즈베리파이에 SSH로 접속해서 아래 명령을 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/user/colcon_ws/install/setup.bash
ros2 run docking dock_turn_backup
```

원격 명령이 exit code `0`으로 종료되는 것을 도킹 성공 신호로 사용합니다. 도킹
성공 후에도 노드가 계속 실행되는 형태라면, 도킹 노드가 성공 시 정상 종료하도록
바꾸거나 별도의 성공 토픽/서비스 연동을 추가해야 합니다.

SSH 접속 확인:

```bash
ssh -i /root/.ssh/id_ed25519_drive_manager user@192.168.0.3
```

도킹 실행 파일 확인:

```bash
ssh -i /root/.ssh/id_ed25519_drive_manager user@192.168.0.3 \
  "bash -lc 'source /opt/ros/jazzy/setup.bash && source /home/user/colcon_ws/install/setup.bash && ros2 pkg executables docking | grep dock_turn_backup'"
```

## 현재 ROS 2 토픽 정리

아래 내용은 실행 중인 ROS 2 그래프에서 다음 명령으로 수집했습니다.

> 0903 변경 이후 라이다의 실제 주행 경로는
> `/scan -> /scan_front_filtered -> AMCL/costmap/Collision Monitor`입니다. 아래 표는
> 특정 실행 시점의 전체 그래프 스냅샷이므로 외부에서 실행한 legacy
> `/scan_to_scan_filter_chain`과 `/scan_filtered`가 포함될 수 있지만, 현재
> `drive_manager`는 `/scan_filtered`를 사용하지 않습니다.

```bash
ros2 topic list -t
ros2 topic info --verbose <topic>
```

참고:

- `Publisher count`는 ROS 2 endpoint 개수입니다. `발행 노드` 목록은 같은 노드명을
  중복 제거해서 정리했습니다.
- `-`는 수집 시점에 발행자가 없었다는 뜻입니다. 구독자가 있으면 발행자가 없어도
  토픽 목록에 표시될 수 있습니다.
- lifecycle, `/parameter_events`, `/rosout`, `/bond` 계열 토픽은 대부분 ROS 2와
  Nav2가 내부 상태 관리, 로그, 진단을 위해 자동으로 만드는 토픽입니다.

| 토픽 | 타입 | Publisher count | 발행 노드 | 역할 |
| --- | --- | ---: | --- | --- |
| `/amcl/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 1 | `/amcl` | AMCL 노드의 lifecycle 상태 전환 이벤트입니다. |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 1 | `/amcl` | AMCL이 추정한 지도 좌표계 기준 로봇 위치와 공분산입니다. |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 1 | `/battery_node` | 로봇 배터리 상태 요약 정보입니다. |
| `/behavior_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 1 | `/behavior_server` | Nav2 behavior server의 lifecycle 상태 전환 이벤트입니다. |
| `/behavior_tree_log` | `nav2_msgs/msg/BehaviorTreeLog` | 2 | `/bt_navigator_navigate_through_poses_rclcpp_node`, `/bt_navigator_navigate_to_pose_rclcpp_node` | Nav2 behavior tree 실행 로그입니다. |
| `/bond` | `bond/msg/Status` | 24 | `/amcl`, `/behavior_server`, `/bt_navigator`, `/collision_monitor`, `/controller_server`, `/docking_server`, `/lifecycle_manager_localization`, `/lifecycle_manager_navigation`, `/map_server`, `/planner_server`, `/route_server`, `/smoother_server`, `/velocity_smoother`, `/waypoint_follower` | Nav2 lifecycle manager와 각 managed node 사이의 heartbeat/bond 상태입니다. |
| `/bt_navigator/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 1 | `/bt_navigator` | BT navigator의 lifecycle 상태 전환 이벤트입니다. |
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 1 | `/camera/camera` | RealSense 컬러 카메라의 보정값과 내부 파라미터입니다. |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 1 | `/camera/camera` | 컬러 카메라의 원본 이미지 스트림입니다. |
| `/camera/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 1 | `/camera/camera` | 압축된 컬러 이미지 스트림입니다. |
| `/camera/camera/color/image_raw/compressedDepth` | `sensor_msgs/msg/CompressedImage` | 1 | `/camera/camera` | 컬러 이미지 토픽에 대한 compressedDepth transport 변형입니다. |
| `/camera/camera/color/image_raw/theora` | `theora_image_transport/msg/Packet` | 1 | `/camera/camera` | Theora 방식으로 인코딩된 컬러 이미지 transport입니다. |
| `/camera/camera/color/image_raw/zstd` | `sensor_msgs/msg/CompressedImage` | 1 | `/camera/camera` | Zstd 방식으로 압축된 컬러 이미지 transport입니다. |
| `/camera/camera/color/metadata` | `realsense2_camera_msgs/msg/Metadata` | 1 | `/camera/camera` | RealSense 컬러 스트림 메타데이터입니다. |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 1 | `/camera/camera` | RealSense depth 카메라의 보정값과 내부 파라미터입니다. |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 1 | `/camera/camera` | 보정된 원본 depth 이미지 스트림입니다. |
| `/camera/camera/depth/image_rect_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 1 | `/camera/camera` | 압축된 depth 이미지 스트림입니다. |
| `/camera/camera/depth/image_rect_raw/compressedDepth` | `sensor_msgs/msg/CompressedImage` | 1 | `/camera/camera` | depth 이미지 전용 compressedDepth transport입니다. |
| `/camera/camera/depth/image_rect_raw/theora` | `theora_image_transport/msg/Packet` | 1 | `/camera/camera` | Theora 방식으로 인코딩된 depth 이미지 transport입니다. |
| `/camera/camera/depth/image_rect_raw/zstd` | `sensor_msgs/msg/CompressedImage` | 1 | `/camera/camera` | Zstd 방식으로 압축된 depth 이미지 transport입니다. |
| `/camera/camera/depth/metadata` | `realsense2_camera_msgs/msg/Metadata` | 1 | `/camera/camera` | RealSense depth 스트림 메타데이터입니다. |
| `/camera/camera/extrinsics/depth_to_color` | `realsense2_camera_msgs/msg/Extrinsics` | 1 | `/camera/camera` | depth 카메라 좌표계에서 컬러 카메라 좌표계로 가는 외부 파라미터입니다. |
| `/client_count` | `std_msgs/msg/Int32` | 1 | `/rosbridge_websocket` | rosbridge에 연결된 클라이언트 수입니다. |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 2 | `/collision_monitor`, `/docking_server` | 베이스 컨트롤러로 전달되는 최종 속도 명령입니다. |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 6 | `/behavior_server`, `/controller_server` | Nav2가 생성한 속도 명령으로, smoothing/collision filtering 전 단계의 명령입니다. |
| `/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | 1 | `/velocity_smoother` | velocity smoother가 보정한 속도 명령입니다. |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | 0 | - | 텔레오퍼레이션 속도 입력 토픽이며, 수집 시점에는 발행자가 없었습니다. |
| `/collision_monitor/collision_points_marker` | `visualization_msgs/msg/MarkerArray` | 1 | `/collision_monitor` | collision monitor의 감시 지점/영역을 RViz에서 보기 위한 marker입니다. |
| `/collision_monitor/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 1 | `/collision_monitor` | collision monitor의 lifecycle 상태 전환 이벤트입니다. |
| `/collision_monitor_state` | `nav2_msgs/msg/CollisionMonitorState` | 1 | `/collision_monitor` | 현재 collision monitor 상태와 동작 정보입니다. |
| `/connected_clients` | `rosbridge_msgs/msg/ConnectedClients` | 1 | `/rosbridge_websocket` | rosbridge에 연결된 클라이언트 상세 목록입니다. |
| `/controller_selector` | `std_msgs/msg/String` | 0 | - | Nav2 controller를 선택하기 위한 입력 토픽이며, 수집 시점에는 발행자가 없었습니다. |
| `/controller_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 1 | `/controller_server` | controller server의 lifecycle 상태 전환 이벤트입니다. |
| `/cost_cloud` | `sensor_msgs/msg/PointCloud2` | 1 | `/controller_server` | local planner/controller 디버깅용 cost cloud입니다. |
| `/detected_dock_pose` | `geometry_msgs/msg/PoseStamped` | 0 | - | dock 감지 결과 pose 입력 토픽이며, 수집 시점에는 발행자가 없었습니다. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 3 | `/lifecycle_manager_localization`, `/lifecycle_manager_navigation`, `/scan_to_scan_filter_chain` | lifecycle manager와 laser filter chain의 진단 정보입니다. |
| `/dock_pose` | `geometry_msgs/msg/PoseStamped` | 1 | `/docking_server` | docking pipeline에서 사용하는 dock 위치입니다. |
| `/docking_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 1 | `/docking_server` | docking server의 lifecycle 상태 전환 이벤트입니다. |
| `/docking_trajectory` | `nav_msgs/msg/Path` | 1 | `/docking_server` | docking 과정에서 생성되거나 실행 중인 경로입니다. |
| `/evaluation` | `dwb_msgs/msg/LocalPlanEvaluation` | 1 | `/controller_server` | DWB local planner의 trajectory 평가/디버깅 정보입니다. |
| `/filtered_dock_pose` | `geometry_msgs/msg/PoseStamped` | 1 | `/docking_server` | 필터링된 dock 위치 추정값입니다. |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | `/global_costmap/global_costmap` | 표준 occupancy grid 형식의 global costmap입니다. |
| `/global_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | 1 | `/global_costmap/global_costmap` | Nav2 raw costmap 형식의 global costmap입니다. |
| `/global_costmap/costmap_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | 1 | `/global_costmap/global_costmap` | global costmap의 raw 증분 업데이트입니다. |
| `/global_costmap/costmap_updates` | `map_msgs/msg/OccupancyGridUpdate` | 1 | `/global_costmap/global_costmap` | global occupancy grid의 증분 업데이트입니다. |
| `/global_costmap/footprint` | `geometry_msgs/msg/Polygon` | 0 | - | global costmap에 넣을 수 있는 footprint 입력이며, 수집 시점에는 발행자가 없었습니다. |
| `/global_costmap/global_costmap/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 1 | `/global_costmap/global_costmap` | global costmap의 lifecycle 상태 전환 이벤트입니다. |
| `/global_costmap/obstacle_layer` | `nav_msgs/msg/OccupancyGrid` | 1 | `/global_costmap/global_costmap` | global costmap의 obstacle layer를 occupancy grid로 표현한 토픽입니다. |
| `/global_costmap/obstacle_layer_raw` | `nav2_msgs/msg/Costmap` | 1 | `/global_costmap/global_costmap` | global costmap obstacle layer의 Nav2 raw costmap입니다. |
| `/global_costmap/obstacle_layer_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | 1 | `/global_costmap/global_costmap` | global costmap obstacle layer의 raw 증분 업데이트입니다. |
| `/global_costmap/obstacle_layer_updates` | `map_msgs/msg/OccupancyGridUpdate` | 1 | `/global_costmap/global_costmap` | global costmap obstacle layer의 occupancy grid 증분 업데이트입니다. |
| `/global_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 1 | `/global_costmap/global_costmap` | global costmap이 현재 사용하는 로봇 footprint입니다. |
| `/global_costmap/static_layer` | `nav_msgs/msg/OccupancyGrid` | 1 | `/global_costmap/global_costmap` | global costmap의 static map layer입니다. |
| `/global_costmap/static_layer_raw` | `nav2_msgs/msg/Costmap` | 1 | `/global_costmap/global_costmap` | global costmap static layer의 Nav2 raw costmap입니다. |
| `/global_costmap/static_layer_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | 1 | `/global_costmap/global_costmap` | global costmap static layer의 raw 증분 업데이트입니다. |
| `/global_costmap/static_layer_updates` | `map_msgs/msg/OccupancyGridUpdate` | 1 | `/global_costmap/global_costmap` | global costmap static layer의 occupancy grid 증분 업데이트입니다. |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 0 | - | navigation goal pose 입력 토픽이며, 수집 시점에는 발행자가 없었습니다. |
| `/imu/data` | `sensor_msgs/msg/Imu` | 1 | `/stella_ahrs_node` | 필터링/융합된 IMU orientation, angular velocity, acceleration 데이터입니다. |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1 | `/stella_ahrs_node` | 원본 IMU 측정값입니다. |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | 1 | `/stella_ahrs_node` | AHRS/IMU의 magnetometer 데이터입니다. |
| `/imu/yaw` | `std_msgs/msg/Float64` | 1 | `/stella_ahrs_node` | AHRS/IMU에서 계산한 yaw 각도입니다. |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 0 | - | AMCL 초기 위치 입력 토픽이며, 보통 RViz나 UI에서 발행합니다. |
| `/joint_states` | `sensor_msgs/msg/JointState` | 1 | `/joint_state_publisher` | robot_state_publisher가 사용할 로봇 joint 상태입니다. |
| `/linear` | `std_msgs/msg/Int32` | 0 | - | 커스텀 linear 명령/상태 토픽으로 보이며, 수집 시점에는 발행자가 없었습니다. |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | `/local_costmap/local_costmap` | 표준 occupancy grid 형식의 local costmap입니다. |
| `/local_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | 1 | `/local_costmap/local_costmap` | Nav2 raw costmap 형식의 local costmap입니다. |
| `/local_costmap/costmap_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | 1 | `/local_costmap/local_costmap` | local costmap의 raw 증분 업데이트입니다. |
| `/local_costmap/costmap_updates` | `map_msgs/msg/OccupancyGridUpdate` | 1 | `/local_costmap/local_costmap` | local occupancy grid의 증분 업데이트입니다. |
| `/local_costmap/footprint` | `geometry_msgs/msg/Polygon` | 0 | - | local costmap에 넣을 수 있는 footprint 입력이며, 수집 시점에는 발행자가 없었습니다. |
| `/local_costmap/local_costmap/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 1 | `/local_costmap/local_costmap` | local costmap의 lifecycle 상태 전환 이벤트입니다. |
| `/local_costmap/obstacle_layer` | `nav_msgs/msg/OccupancyGrid` | 1 | `/local_costmap/local_costmap` | local costmap의 obstacle layer를 occupancy grid로 표현한 토픽입니다. |
| `/local_costmap/obstacle_layer_raw` | `nav2_msgs/msg/Costmap` | 1 | `/local_costmap/local_costmap` | local costmap obstacle layer의 Nav2 raw costmap입니다. |
| `/local_costmap/obstacle_layer_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | 1 | `/local_costmap/local_costmap` | local costmap obstacle layer의 raw 증분 업데이트입니다. |
| `/local_costmap/obstacle_layer_updates` | `map_msgs/msg/OccupancyGridUpdate` | 1 | `/local_costmap/local_costmap` | local costmap obstacle layer의 occupancy grid 증분 업데이트입니다. |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 1 | `/local_costmap/local_costmap` | local costmap이 현재 사용하는 로봇 footprint입니다. |
| `/local_costmap/static_layer` | `nav_msgs/msg/OccupancyGrid` | 1 | `/local_costmap/local_costmap` | local costmap의 static map layer입니다. |
| `/local_costmap/static_layer_raw` | `nav2_msgs/msg/Costmap` | 1 | `/local_costmap/local_costmap` | local costmap static layer의 Nav2 raw costmap입니다. |
| `/local_costmap/static_layer_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | 1 | `/local_costmap/local_costmap` | local costmap static layer의 raw 증분 업데이트입니다. |
| `/local_costmap/static_layer_updates` | `map_msgs/msg/OccupancyGridUpdate` | 1 | `/local_costmap/local_costmap` | local costmap static layer의 occupancy grid 증분 업데이트입니다. |
| `/local_plan` | `nav_msgs/msg/Path` | 1 | `/controller_server` | controller가 선택한 local plan입니다. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 1 | `/map_server` | 정적 occupancy grid map입니다. |
| `/map_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 1 | `/map_server` | map server의 lifecycle 상태 전환 이벤트입니다. |
| `/marker` | `visualization_msgs/msg/MarkerArray` | 1 | `/controller_server` | controller 디버깅/시각화용 marker입니다. |
| `/odom` | `nav_msgs/msg/Odometry` | 1 | `/stella_md_node` | STELLA motor driver node가 발행하는 wheel/base odometry입니다. |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | 35 | `/_ros2cli_daemon_0_a04c8ca4259e47a9af17c72f4202c7e8`, `/_ros2cli_daemon_0_fcf273c59b8b4153bf5bbee0a0a373e0`, `/amcl`, `/battery_node`, `/behavior_server`, `/bt_navigator`, `/bt_navigator_navigate_through_poses_rclcpp_node`, `/bt_navigator_navigate_to_pose_rclcpp_node`, `/camera/camera`, `/collision_monitor`, `/controller_server`, `/docking_server`, `/global_costmap/global_costmap`, `/joint_state_publisher`, `/launch_ros_529766`, `/launch_ros_595732`, `/lifecycle_manager_localization`, `/lifecycle_manager_navigation`, `/linear_motor_node`, `/local_costmap/local_costmap`, `/map_server`, `/planner_server`, `/robot_state_publisher`, `/rosapi`, `/rosbridge_websocket`, `/route_server`, `/scan_to_scan_filter_chain`, `/sllidar2_node`, `/sllidar_node`, `/smoother_server`, `/stella_ahrs_node`, `/stella_md_node`, `/velocity_smoother`, `/waypoint_follower` | ROS 2 파라미터 변경/이벤트 알림입니다. |
| `/particle_cloud` | `nav2_msgs/msg/ParticleCloud` | 1 | `/amcl` | AMCL localization 디버깅용 particle cloud입니다. |
| `/plan` | `nav_msgs/msg/Path` | 2 | `/planner_server`, `/route_server` | global navigation path 또는 route plan입니다. |
| `/plan_smoothed` | `nav_msgs/msg/Path` | 1 | `/smoother_server` | smoother server가 보정한 global plan입니다. |
| `/planner_selector` | `std_msgs/msg/String` | 0 | - | Nav2 planner를 선택하기 위한 입력 토픽이며, 수집 시점에는 발행자가 없었습니다. |
| `/planner_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 1 | `/planner_server` | planner server의 lifecycle 상태 전환 이벤트입니다. |
| `/preempt_teleop` | `std_msgs/msg/Empty` | 0 | - | teleop 선점/preempt 트리거 토픽이며, 수집 시점에는 발행자가 없었습니다. |
| `/received_global_plan` | `nav_msgs/msg/Path` | 1 | `/controller_server` | controller server가 받은 global plan입니다. |
| `/robot_description` | `std_msgs/msg/String` | 2 | `/robot_state_publisher` | URDF 로봇 모델 설명입니다. |
| `/rosout` | `rcl_interfaces/msg/Log` | 38 | `/_ros2cli_daemon_0_a04c8ca4259e47a9af17c72f4202c7e8`, `/_ros2cli_daemon_0_fcf273c59b8b4153bf5bbee0a0a373e0`, `/amcl`, `/battery_node`, `/behavior_server`, `/bt_navigator`, `/bt_navigator_navigate_through_poses_rclcpp_node`, `/bt_navigator_navigate_to_pose_rclcpp_node`, `/camera/camera`, `/collision_monitor`, `/controller_server`, `/docking_server`, `/global_costmap/global_costmap`, `/joint_state_publisher`, `/launch_ros_529766`, `/launch_ros_595732`, `/lifecycle_manager_localization`, `/lifecycle_manager_navigation`, `/linear_motor_node`, `/local_costmap/local_costmap`, `/map_server`, `/nav2_container`, `/planner_server`, `/robot_state_publisher`, `/rosapi`, `/rosbridge_websocket`, `/route_server`, `/scan_to_scan_filter_chain`, `/sllidar2_node`, `/sllidar_node`, `/smoother_server`, `/stella_ahrs_node`, `/stella_md_node`, `/transform_listener_impl_ffff64008ad0`, `/transform_listener_impl_ffff7c004360`, `/velocity_smoother`, `/waypoint_follower` | 실행 중인 ROS 2 노드들의 통합 로그 출력입니다. |
| `/route_graph` | `visualization_msgs/msg/MarkerArray` | 1 | `/route_server` | route server의 route graph 시각화 marker입니다. |
| `/route_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 1 | `/route_server` | route server의 lifecycle 상태 전환 이벤트입니다. |
| `/scan` | `sensor_msgs/msg/LaserScan` | 1 | `/sllidar_node` | 첫 번째 SLLIDAR의 laser scan입니다. |
| `/scan_2` | `sensor_msgs/msg/LaserScan` | 1 | `/sllidar2_node` | 두 번째 SLLIDAR의 laser scan입니다. |
| `/scan_front_filtered` | `sensor_msgs/msg/LaserScan` | 1 | `/front_scan_filter` | 0903 이후 AMCL, costmap, Collision Monitor, Cartographer가 사용하는 전방 +/-120° 필터 스캔입니다. |
| `/scan_filtered` | `sensor_msgs/msg/LaserScan` | 1 | `/scan_to_scan_filter_chain` | scan filter chain을 거친 필터링 laser scan입니다. |
| `/sk120/available` | `sensor_msgs/msg/BatteryState` | 1 | `/battery_node` | SK120 전원 공급 장치/배터리 사용 가능 상태입니다. |
| `/sk120/cmd_output` | `std_msgs/msg/Bool` | 0 | - | SK120 output enable/disable 명령 입력이며, 수집 시점에는 발행자가 없었습니다. |
| `/sk120/current_out` | `std_msgs/msg/Float32` | 1 | `/battery_node` | SK120 출력 전류 측정값입니다. |
| `/sk120/current_set` | `std_msgs/msg/Float32` | 1 | `/battery_node` | SK120 전류 설정값입니다. |
| `/sk120/output_on` | `std_msgs/msg/Bool` | 1 | `/battery_node` | SK120 출력이 켜져 있는지 나타내는 상태입니다. |
| `/sk120/voltage_out` | `std_msgs/msg/Float32` | 1 | `/battery_node` | SK120 출력 전압 측정값입니다. |
| `/smoother_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 1 | `/smoother_server` | smoother server의 lifecycle 상태 전환 이벤트입니다. |
| `/speed_limit` | `nav2_msgs/msg/SpeedLimit` | 1 | `/route_server` | Nav2 navigation 구성요소에서 사용하는 속도 제한 정보입니다. |
| `/staging_pose` | `geometry_msgs/msg/PoseStamped` | 1 | `/docking_server` | 최종 docking 전에 접근할 staging pose입니다. |
| `/stella_ahrs_node/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 0 | - | STELLA AHRS node의 lifecycle transition 토픽이며, 수집 시점에는 발행자가 없었습니다. |
| `/stella_md_node/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 0 | - | STELLA motor driver node의 lifecycle transition 토픽이며, 수집 시점에는 발행자가 없었습니다. |
| `/tf` | `tf2_msgs/msg/TFMessage` | 4 | `/amcl`, `/robot_state_publisher`, `/stella_md_node` | odom/map/base link 등을 포함한 동적 좌표 변환입니다. |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | `/camera/camera`, `/robot_state_publisher` | 로봇과 카메라 고정 프레임 등을 포함한 정적 좌표 변환입니다. |
| `/transformed_global_plan` | `nav_msgs/msg/Path` | 1 | `/controller_server` | controller/local frame 기준으로 변환된 global plan입니다. |
| `/velocity_smoother/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 1 | `/velocity_smoother` | velocity smoother의 lifecycle 상태 전환 이벤트입니다. |
| `/waypoint_follower/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 1 | `/waypoint_follower` | waypoint follower의 lifecycle 상태 전환 이벤트입니다. |
