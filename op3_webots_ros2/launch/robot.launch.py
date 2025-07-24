import os, subprocess
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, EmitEvent, OpaqueFunction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from ament_index_python.packages import get_package_share_directory

def spawn_controller(context, webots_action, gain_file):
    # Get Webots PID
    try:
        pid = webots_action.process_details[0].pid
    except Exception:
        pid = int(subprocess.check_output(['pgrep', '-n', 'webots']).decode().strip())

    return [Node(
        package="op3_webots_ros2",
        executable="op3_extern_controller",
        name="controller_op3",
        output="screen",
        parameters=[{"gain_file_path": gain_file}],
        additional_env={ 'WEBOTS_CONTROLLER_URL': 'tcp://localhost:1234/ROBOTIS_OP3_0'},
    )]

def generate_launch_description():
    pkg = get_package_share_directory("op3_webots_ros2")
    world = os.path.join(pkg, "worlds", "field.wbt")
    gain_file = os.path.join(pkg, "resource", "op3_pid_gain_default.yaml")

    webots = WebotsLauncher(world=world)

    start_ctrl = RegisterEventHandler(
        OnProcessStart(
            target_action=webots,
            on_start=[TimerAction(
                period=2.0,
                actions=[OpaqueFunction(function=spawn_controller,
                                        args=(webots, gain_file))]
            )]
        )
    )

    shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=webots,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    return LaunchDescription([webots, start_ctrl, shutdown])

