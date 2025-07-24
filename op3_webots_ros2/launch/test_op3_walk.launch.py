import os
from launch import LaunchDescription
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

def generate_launch_description():
    args_arg = DeclareLaunchArgument(
        "args",
        default_value="",
        description="Arguments passed to op3_gui_demo"
    )

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="false",
        description="Launch GUI tools (RViz, GUI demo, joint state publisher)"
    )
    
    package_dir = get_package_share_directory("op3_webots_ros2")
    simulation_default = True
    simulation_robot_name_default = "robotis_op3"
    offset_file_path_default = package_dir + "/config/offset.yaml"
    robot_file_path_default = package_dir + "/config/OP3.robot"
    init_file_path_default = package_dir + "/config/dxl_init_OP3.yaml"
    gain_file_path_default = package_dir + "/resource/op3_pid_gain_default.yaml"
    op3_description_path = FindPackageShare("op3_description")
    op3_urdf_path = PathJoinSubstitution([op3_description_path, "urdf", "robotis_op3.urdf.xacro"])
    rviz_config_path = PathJoinSubstitution([op3_description_path, "rviz", "op3.rviz"])
    op3_behavior_path = get_package_share_directory("robocup_behavior")

    webots = WebotsLauncher(
        world=os.path.join(package_dir, "worlds", "test_op3_walk.wbt")
    )

    op3_controller_node = Node(
        package="op3_webots_ros2",
        executable="op3_extern_controller",
        name="controller_op3_0",
        output="screen",
        parameters=[{"gain_file_path": gain_file_path_default}],
        additional_env={
            "WEBOTS_CONTROLLER_URL": "ROBOTIS_OP3_0",
            "ROS_DOMAIN_ID": "0",
        },
    )

    op3_manager_node = Node(
        package="op3_webots_ros2",
        executable="op3_manager",
        # name='op3_manager',
        output="screen",
        parameters=[
            {
                "simulation": simulation_default,
                "simulation_robot_name": simulation_robot_name_default,
                "offset_file_path": offset_file_path_default,
                "robot_file_path": robot_file_path_default,
                "init_file_path": init_file_path_default,            
            }
        ],
        additional_env={"ROS_DOMAIN_ID": "0"},
    )
    op3_manager_launch = TimerAction(
        period=8.0,  # Delay in seconds
        actions=[op3_manager_node],
    )

    robot_description = ParameterValue(Command(["xacro", " ", op3_urdf_path]), value_type=str)

    joint_state_publisher_gui = Node(
        condition=IfCondition(LaunchConfiguration("gui")),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{"source_list": ["/robotis/present_joint_states"]}],
        remappings=[("/joint_states", "/robotis/present_joint_states")]
    )

    robot_state_publisher = Node(
        condition=IfCondition(LaunchConfiguration("gui")),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        remappings=[("/joint_states", "/robotis/present_joint_states")]
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("gui")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path]
    )

    op3_gui_demo_node = Node(
        condition=IfCondition(LaunchConfiguration("gui")),
        package="op3_gui_demo",
        executable="op3_gui_demo",
        name="op3_demo_opc",
        output="screen",
        parameters=[{
            "demo_config": PathJoinSubstitution([
                FindPackageShare("op3_gui_demo"),
                "config",
                "gui_config.yaml"
            ])
        }],
        arguments=[LaunchConfiguration("args")],
        remappings=[
            ("/op3_demo/ik_target_pose", "/pose_panel/pose")
        ]
    )

    behavior_node = Node(
        condition=UnlessCondition(LaunchConfiguration("gui")),
        package='robocup_behavior',
        executable='bt',
        name='robocup_behavior_bt',
        output='screen',
        parameters=[
            os.path.join(op3_behavior_path, 'config', 'bt.yaml'),
            {'use_sim_time': True},
            {'behavior': 'test_walk'},
        ]
    )

    behavior_launch = TimerAction(
        period=12.0,  # Delay in seconds
        actions=[behavior_node],
    )

    return LaunchDescription([
        gui_arg,
        args_arg,
        webots,
        op3_controller_node,
        op3_manager_launch,
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz_node,
        op3_gui_demo_node,
        behavior_launch
    ])
