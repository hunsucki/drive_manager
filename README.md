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
