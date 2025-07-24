from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    blue_team_arg = DeclareLaunchArgument(
        "blue_team_number",
        default_value="0",
        description="Team number for the blue team"
    )

    red_team_arg = DeclareLaunchArgument(
        "red_team_number",
        default_value="1",
        description="Team number for the red team"
    )

    simulation_default = True
    simulation_robot_name_default = "robotis_op3"
    offset_file_path_default = (
        get_package_share_directory("op3_webots_ros2") + "/config/offset.yaml"
    )
    robot_file_path_default = (
        get_package_share_directory("op3_webots_ros2") + "/config/OP3.robot"
    )
    init_file_path_default = (
        get_package_share_directory("op3_webots_ros2") + "/config/dxl_init_OP3.yaml"
    )
    device_name_default = "/dev/ttyUSB0"

    ld = LaunchDescription([blue_team_arg, red_team_arg])
    num_robots = 3

    for i in range(num_robots):
        team_number = LaunchConfiguration("blue_team_number") #TODO:Add red team support

        ld.add_action(
            Node(
                package="op3_webots_ros2",
                executable="op3_manager",
                output="screen",
                parameters=[
                    {
                        "simulation": simulation_default,
                        "simulation_robot_name": simulation_robot_name_default,
                        "offset_file_path": offset_file_path_default,
                        "robot_file_path": robot_file_path_default,
                        "init_file_path": init_file_path_default,
                        "team_number": team_number,
                        "player_number": i + 1,
                    }
                ],
                additional_env={"ROS_DOMAIN_ID": str(i)},
            )
        )

    return ld
