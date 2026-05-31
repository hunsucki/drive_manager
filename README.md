# drive_manager

헤드리스 Nav2 실행과 로봇 주행 미션 노드를 위한 ROS 2 패키지입니다.

## Headless Nav2 실행

```bash
source /root/colcon_ws/install/setup.bash
ros2 launch drive_manager nav2_headless.launch.py
```

To inspect Nav2 from the Jetson GUI, start RViz with the same launch:

```bash
source /root/colcon_ws/install/setup.bash
ros2 launch drive_manager nav2_headless.launch.py use_rviz:=true
```

이 launch 파일은 패키지 안의 URDF를 사용해 `robot_state_publisher`를 실행하고,
`drive_manager`에 포함된 STELLA Nav2 파라미터로 `nav2_bringup`을 함께 실행합니다.

## Two Point Mission 실행

```bash
source /root/colcon_ws/install/setup.bash
ros2 run drive_manager two_point
```

## Mobile Mission 실행

`drive_manager.launch.py`는 모바일 앱 연동에 필요한 세 노드를 함께 실행합니다.

- `rosbridge_websocket`: 모바일 앱 WebSocket 연결용입니다. 기본 포트는 `9090`입니다.
- `command_manager`: 앱 명령 `/robot_command`를 받아 내부 명령 `/mission_command`로 전달합니다.
- `mission_driver`: `param/mission_config.yaml`을 읽고 Nav2 `/navigate_to_pose` 액션으로 실제 주행을 수행합니다.

```bash
source /root/colcon_ws/install/setup.bash
ros2 launch drive_manager drive_manager.launch.py
```

실제 주행 전에는 Nav2가 먼저 실행되어 있어야 합니다.

```bash
source /root/colcon_ws/install/setup.bash
ros2 launch drive_manager nav2_headless.launch.py
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

명령 동작:

- `START`: dock 출발 escape 후, home_to_patrol_pose로 이동하고, patrol_points를 순회한 뒤 home_to_dock_pose로 돌아와 SSH 도킹 명령을 실행합니다.
- `HOME`: 어디에 있든 현재 미션을 중단하고 home_to_dock_pose로 이동한 뒤 SSH 도킹 명령을 실행합니다.
- `STOP`: 현재 Nav2 goal 또는 도킹 SSH 프로세스를 중단하고 `/cmd_vel` 0을 발행합니다.
- `ESTOP`: STOP과 같지만 latch 상태가 되어 `RESET` 전까지 START/HOME을 거부합니다.
- `RESET`: ESTOP latch를 해제합니다.

START 중 START를 다시 누르면 `mission_driver`가 `BUSY` 상태를 발행하고 기존 미션을 계속 진행합니다. START 중 HOME을 누르면 현재 goal을 취소하고 HOME 복귀 미션으로 전환합니다.

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
    docking_ssh_host: "192.168.0.13"
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
START_ESCAPE
-> HOME_TO_PATROL
-> patrol_points 순서대로 순회
-> HOME_TO_DOCK
-> SSH docking command
```

`start_escape_*`는 도킹스테이션에서 바로 Nav2를 시작할 때 collision_monitor가 막는 상황을 피하기 위한 짧은 탈출 동작입니다. 현재 설정은 `/cmd_vel`로 0.1 m/s 전진을 2초 수행한 뒤 Nav2 주행을 시작합니다.

## SSH 도킹

도킹 단계에서는 `mission_driver`가 라즈베리파이에 SSH로 접속해서 아래 명령을 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/user/colcon_ws/install/setup.bash
ros2 run docking dock_turn_backup
```

SSH 접속 확인:

```bash
ssh -i /root/.ssh/id_ed25519_drive_manager user@192.168.0.13
```

도킹 실행 파일 확인:

```bash
ssh -i /root/.ssh/id_ed25519_drive_manager user@192.168.0.13 \
  "bash -lc 'source /opt/ros/jazzy/setup.bash && source /home/user/colcon_ws/install/setup.bash && ros2 pkg executables docking | grep dock_turn_backup'"
```

## 현재 ROS 2 토픽 정리

아래 내용은 실행 중인 ROS 2 그래프에서 다음 명령으로 수집했습니다.

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
