import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_state = LaunchConfiguration("publish_robot_state").perform(context)
    urdf_model = LaunchConfiguration("urdf_model").perform(context)

    if publish_robot_state.lower() not in ("true", "1", "yes"):
        return []

    with open(urdf_model, "r") as urdf_file:
        robot_description = urdf_file.read()

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=namespace,
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "robot_description": robot_description,
                }
            ],
        )
    ]


def generate_launch_description():
    drive_manager_dir = get_package_share_directory("drive_manager")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_launch_dir = os.path.join(nav2_bringup_dir, "launch")

    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    slam = LaunchConfiguration("slam")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    use_localization = LaunchConfiguration("use_localization")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    default_params_file = os.path.join(drive_manager_dir, "param", "stella.yaml")
    default_urdf_model = os.path.join(drive_manager_dir, "urdf", "stella.urdf")
    default_map_file = os.path.join(drive_manager_dir, "map", "map.yaml")
    default_rviz_config_file = os.path.join(
        drive_manager_dir,
        "rviz",
        "drive_manager_nav2.rviz",
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, "bringup_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "slam": slam,
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
            "log_level": log_level,
            "use_localization": use_localization,
        }.items(),
    )

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace"),
            DeclareLaunchArgument(
                "use_namespace",
                default_value="false",
                description="Whether to apply a namespace to the navigation stack",
            ),
            DeclareLaunchArgument("slam", default_value="False", description="Whether to run SLAM"),
            DeclareLaunchArgument(
                "map",
                default_value=default_map_file,
                description="Full path to the map yaml file to load",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Full path to the Nav2 parameters file",
            ),
            DeclareLaunchArgument(
                "urdf_model",
                default_value=default_urdf_model,
                description="Full path to the robot URDF file",
            ),
            DeclareLaunchArgument(
                "publish_robot_state",
                default_value="false",
                description="Whether this launch should publish robot_description and fixed TF",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically start the Nav2 stack",
            ),
            DeclareLaunchArgument(
                "use_composition",
                default_value="True",
                description="Whether to use composed Nav2 bringup",
            ),
            DeclareLaunchArgument(
                "use_respawn",
                default_value="False",
                description="Whether to respawn nodes if they crash",
            ),
            DeclareLaunchArgument("log_level", default_value="info", description="Log level"),
            DeclareLaunchArgument(
                "use_localization",
                default_value="True",
                description="Whether to enable localization",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Whether to start RViz2 with this launch",
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=default_rviz_config_file,
                description="Full path to the RViz config file",
            ),
            OpaqueFunction(function=launch_setup),
            LogInfo(msg=["Using map file: ", map_yaml_file]),
            LogInfo(msg=["Using params file: ", params_file]),
            bringup_cmd,
            rviz_cmd,
        ]
    )
