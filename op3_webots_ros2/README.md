## Development stuff
https://emanual.robotis.com/docs/en/platform/op3/simulation/#gazebo-installation

https://docs.ros.org/en/kinetic/api/op3_walking_module_msgs/html/msg/WalkingParam.html

https://emanual.robotis.com/docs/en/platform/op3/robotis_ros_packages/

```bash
ros2 topic pub --once /robotis/walking/set_params op3_walking_module_msgs/msg/WalkingParam "{
  init_x_offset: -0.02,
  init_y_offset: 0.015,
  init_z_offset: 0.035,
  init_roll_offset: 0.0,
  init_pitch_offset: 0.0,
  init_yaw_offset: 0.0,
  period_time: 0.65,
  dsp_ratio: 0.2,
  step_fb_ratio: 0.28,
  x_move_amplitude: -0.03,
  y_move_amplitude: 0.0,
  z_move_amplitude: 0.0,
  angle_move_amplitude: 0.0,
  move_aim_on: false,
  balance_enable: true,
  balance_hip_roll_gain: 0.35,
  balance_knee_gain: 0.3,
  balance_ankle_roll_gain: 0.7,
  balance_ankle_pitch_gain: 0.9,
  y_swap_amplitude: 0.028,
  z_swap_amplitude: 0.006,
  arm_swing_gain: 0.2,
  pelvis_offset: 0.0,
  hip_pitch_offset: 0.08,
  p_gain: 0,
  i_gain: 0,
  d_gain: 0
}"

ros2 service call /robotis/set_present_ctrl_modules robotis_controller_msgs/srv/SetModule "{module_name: 'walking_module'}"

ros2 topic pub --once /robotis/walking/command std_msgs/msg/String "data: 'start'"

ros2 topic pub --once /robotis/walking/command std_msgs/msg/String "data: 'stop'"
```


```bash
ros2 topic pub --once /robotis/walking/set_params op3_walking_module_msgs/msg/WalkingParam "{
  init_x_offset: -0.02,
  init_y_offset: 0.015,
  init_z_offset: 0.035,
  init_roll_offset: 0.0,
  init_pitch_offset: 0.0,
  init_yaw_offset: 0.0,
  period_time: 0.9,
  dsp_ratio: 0.2,
  step_fb_ratio: 0.28,
  x_move_amplitude: -0.03,
  y_move_amplitude: 0.0,
  z_move_amplitude: 0.0,
  angle_move_amplitude: 0.0,
  move_aim_on: false,
  balance_enable: true,
  balance_hip_roll_gain: 0.35,
  balance_knee_gain: 0.3,
  balance_ankle_roll_gain: 0.7,
  balance_ankle_pitch_gain: 0.9,
  y_swap_amplitude: 0.028,
  z_swap_amplitude: 0.006,
  arm_swing_gain: 0.2,
  pelvis_offset: 0.0,
  hip_pitch_offset: 0.08,
  p_gain: 0,
  i_gain: 0,
  d_gain: 0
}"

# Walk the robot with GUI
ros2 launch op3_webots_ros2 test_op3_walk.launch.py gui:=true

# Walk the robot with teleop
ros2 launch op3_webots_ros2 test_op3_walk.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Walk the robot with BT
ros2 launch op3_webots_ros2 test_op3_walk.launch.py
# ros2 launch robocup_behavior test_launch.py

## Launch the simulation and robots

```bash
# 1
ros2 launch op3_webots_ros2 robot.launch.py
# 2
ros2 launch op3_webots_ros2 op3_simulation.launch.py
#3
ros2 launch op3_webots_ros2 game_controller.launch.py
```

Options:
  -c, --competition <COMPETITION>  Set the competition type
      --play-off                   Set whether this is a play-off (long) game
      --home-team <HOME_TEAM>      Set the home team (name or number)
      --away-team <AWAY_TEAM>      Set the away team (name or number)
      --no-delay                   Set the no-delay test flag
      --penalty-shootout           Set the penalty shoot-out test flag
      --unpenalize                 Set the unpenalize test flag
  -f, --fullscreen                 Open the main window in fullscreen mode
  -i, --interface <INTERFACE>      Set the network interface to listen/send on/to
  -b, --broadcast                  Send control messages to 255.255.255.255
  -m, --multicast                  Join multicast groups for simulated team communication
      --sync                       Sync the log file to the storage device after each entry
      --replay <REPLAY>            Specify the path to a log file to replay
  -l, --league                     the League the controller is for
  -h, --help                       Print help
  -V, --version                    Print version
  