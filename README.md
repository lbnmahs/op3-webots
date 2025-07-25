# OP3 + Webots: A Scaffolded Learning Path for Beginners

This is a work in progress... 

> **Audience**: Students with limited ROS 2/Webots/robotics experience
>
> **Goal**: By the end, students can launch the OP3 in Webots, inspect and use the OP3 framework (modules, topics, services), and make simple behaviour changes (e.g., start/stop walking, tweak parameters, write a small node).

---

## How to Use This Pack

* **Each lesson builds on the previous one.** Don’t skip steps unless your group is confident.
* Every lesson includes: **Learning Goals, Concepts, Step-by-Step Tasks, Checkpoints, Troubleshooting, and Extensions**.

---

## Overview of the Learning Path

| Lesson | Title                                             | Core Skill                     | Outcome                                         |
| ------ | ------------------------------------------------- | ------------------------------ | ----------------------------------------------- |
| 0      | Environment & Workspace Setup                     | Install & build                | Working ROS 2 + Webots + OP3 workspace          |
| 1      | First Launch in Webots                            | Launch files & simulation time | OP3 world running; students can see/echo topics |
| 2      | Exploring the OP3 Framework                       | Topics, services, params       | Students enable a module and read joint states  |
| 3      | Make the Robot Walk                               | Publish/Service calls          | Robot walks in sim via CLI commands             |
| 4      | Tune Parameters & Observe Effects                 | Parameters & feedback          | Students adjust walking amplitudes safely       |
| 5      | Write a Tiny Control Node                         | ROS 2 publisher/service client | Python node that starts/stops walking           |
| 6      | Behaviour Trees / Higher-Level Control (Optional) | BTs / demos                    | Launch BT demo or create a small BT tweak       |
| 7      | Extend the Framework (Optional Advanced)          | Modules/plugins                | Read code, add a trivial feature/module         |

---

## Lesson 1 – Environment & Workspace Setup

**Learning Goals**

* Use git repository to clone/build OP3 packages.
* Understand what a ROS 2 workspace is.
* Launch the OP3 robot in Webots.

**Key Concepts**: ROS 2 distributions, colcon workspace, sourcing.

**Steps**

1. **Install prerequisites** (OS packages, ROS 2, Webots).
2. **Quick setup using op3-meta repo**:
    This script clones all required repos and prepares the OP3 ROS2 workspace as described in its README.

    ```bash
    mkdir -p robotis_ws/src
    cd robotis_ws/src
    git clone https://github.com/blueshrapnel/op3-meta.git
    cd ..
    ./src/op3-meta/setup.sh
    ```

3. **Build & source**:

   ```bash
   # ensure that you are in the top level robotis_ws folder
   colcon build --symlink-install
   source install/setup.bash

   # you also have to source the core ros2 files
   source /opt/ros/jazzy/setup.sh

   # when you are finished building you should have the following folders:
   └── robotis_ws
       ├── build
       ├── install
       ├── log
       └── src
   ```

4. **Launch the robot in Webots**:

    ```bash
    # install the ros webots package for jazzy 
    sudo apt-get install ros-jazzy-webots-ros2

    # launch the OP3 robot in webots
    ros2 launch op3_webots_ros2 robot_launch.py
    ```

5. **Investigate ros2 pkg commands**:
    What is a ROS 2 package? It is the smallest buildable/releasable unit in ROS 2. A package bundles code, interfaces (msgs/srvs), launch/param files, plus a `package.xml` and build files so tools like colcon know how to build/install it.
    
    For example,  `ros2 pkg list` shows all packages your environment can currently see. Other handy commands:
    
    ```bash
    ros2 pkg prefix <name>  # where it is installed
    ros2 pkg xml <name>     # print its metadata
    ```

**Checkpoints**

* You can run `ros2 pkg list | grep msgs` and see the packages related to messages.
* Want more depth? Check the official ROS 2 tutorials on "Creating a package" and the Packages overview in the ROS 2 docs.
* Students can launch Webots (even just the default world) to confirm it runs.

**Troubleshooting**

* Missing deps → run `rosdep` again.
    ```bash
    # from the the top level ros workspace folder
    rosdep install --from-paths src --ignore-src -r -y
    ```
* Build errors → read the first error line; comment with full text and ask for help.

**Extension**

* Create an alias  that sources the workspace, and another that sources ros core files.  You could add this to your `.bash_aliases`  or `.zsh_aliases` file depending on which shell you are using.

  ```bash
  # ros2 aliases
  alias sr='source /opt/ros/${ROS_DISTRO}/setup.sh'
  alias sl='source install/local_setup.sh'
  alias ss='source install/setup.sh'
  ```

---

\## Lesson 2 – First Launch in Webots

\*\*Learning Goals\*\*

\- Start the OP3 Webots world with a provided launch file.

\- Recognise when nodes start too early (timing issues).

\*\*Concepts\*\*: Launch files, processes, temp folders (\`/tmp/webots\`), \`TimerAction\`.


### Big‑picture architecture

The tree structure below shows the contents of your ROS2 workspace once you have
built the packages.

```bash
workspace/
.
├── build
│   ├── COLCON_IGNORE
│   ├── dynamixel_sdk
│   └── …     
├── log
│   ├── build_2025-07-23_21-55-07
│   │   ├── dynamixel_sdk
│   │   ├── dynamixel_sdk_custom_interfaces
│   └── …     
├── install
│   ├── COLCON_IGNORE
│   ├── dynamixel_sdk
│   │   ├── include
│   │   ├── lib
│   │   └── share
│   └── …
└── src
    ├── DynamixelSDK
    │   ├── c
    │   ├── c#
    │   ├── c++
    │   ├── documents
    │   ├── Doxyfile
    │   ├── java
    │   ├── labview
    │   ├── LICENSE
    │   ├── matlab
    │   ├── python
    │   ├── README.md
    │   ├── ReleaseNote.md
    │   └── ros
    ├── op3-meta
    │   ├── op3.repos
    │   ├── readme.md
    │   └── setup.sh
    ├── op3-webots
    │   ├── op3_webots_ros2                                  ←  rewrite simulation interface package
    │   └── README.md
    ├── ROBOTIS-Framework
    │   ├── LICENSE
    │   ├── README.md
    │   ├── robotis_controller
    │   ├── robotis_device
    │   └── robotis_framework_common
    ├── ROBOTIS-Framework-msgs
    │   ├── LICENSE
    │   ├── README.md
    │   └── robotis_controller_msgs
    ├── ROBOTIS-Math
    │   ├── LICENSE
    │   ├── README.md
    │   └── robotis_math
    ├── ROBOTIS-OP3
    │   ├── LICENSE
    │   ├── op3_action_module
    │   ├── op3_balance_control
    │   ├── op3_base_module
    │   ├── op3_direct_control_module
    │   ├── op3_head_control_module
    │   ├── op3_kinematics_dynamics
    │   ├── op3_localization
    │   ├── op3_manager
    │   ├── op3_online_walking_module
    │   ├── op3_tuning_module
    │   ├── op3_walking_module
    │   ├── open_cr_module
    │   └── README.md
    ├── ROBOTIS-OP3-Common
    │   ├── LICENSE
    │   ├── op3_description
    │   └── README.md
    ├── ROBOTIS-OP3-Demo
    │   ├── LICENSE
    │   ├── op3_ball_detector
    │   ├── op3_bringup
    │   ├── op3_demo
    │   ├── op3_read_write_demo
    │   └── README.md
    ├── ROBOTIS-OP3-msgs
    │   ├── LICENSE
    │   ├── op3_action_module_msgs
    │   ├── op3_ball_detector_msgs
    │   ├── op3_camera_setting_tool_msgs
    │   ├── op3_offset_tuner_msgs
    │   ├── op3_online_walking_module_msgs
    │   ├── op3_tuning_module_msgs
    │   ├── op3_walking_module_msgs
    │   └── README.md
    └── ROBOTIS-OP3-Simulations                              ←  original OP3 simulation interface package
        ├── COLCON_IGNORE
        ├── LICENSE
        ├── op3_gazebo_ros2
        ├── op3_webots_ros2
        ├── README.md
        └── rf_gz_bridge
```

For this section we will be focussing on the package that launches the robot and
required nodes for walking, etc. in webots.  This is our new package `op3-webots`.

```bash
op3-webots
├── op3_webots_ros2                                          ←  rewrite simulation interface package
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── domain_bridge.yaml
│   │   ├── dxl_init_OP3.yaml
│   │   ├── offset.yaml
│   │   └── OP3.robot
│   ├── controllers
│   ├── images
│   │   └── grid.png
│   ├── include 
│   │   └── op3_webots_ros2                                 ← header files (.hpp)
│   ├── launch
│   │   ├── op3_simulation.launch.py
│   │   ├── robot.launch.py                                 ← starts webots and robot
│   │   └── test_op3_walk.launch.py
│   ├── LICENSE
│   ├── package.xml
│   ├── protos
│   │   ├── backgrounds
│   │   ├── ball_textures
│   │   ├── lighting
│   │   ├── RobocupGoal.proto
│   │   ├── RobocupSoccerField.proto
│   │   ├── RobotisOp3.proto
│   │   └── textures
│   ├── README.md
│   ├── resource
│   │   ├── op3_pid_gain_default.yaml                       ← PID gains
│   │   ├── op3.urdf
│   │   └── ros2_control.yml
│   ├── src
│   │   ├── extern_controller_main.cpp                      ← sets up env + spins controller
│   │   ├── op3_extern_controller.cpp                       ← talks to Webots (C++)
│   └── worlds
│       ├── 3_op_robots.wbt
│       ├── adult.wbt
│       ├── field.wbt
│       ├── op3_soccer.wbt
│       └── test_op3_walk.wbt
└── README.md

```
---
## Launch the simulator, robot and control nodes manually

The steps below assume that the current working directory is the ROS workspace, i.e.
`~/robotis_ws`.  In every new terminal make sure that you source both the ros core
files and your workspace files.  If you created aliases, then use those.  

```bash
source /opt/ros/jazzy/setup.bash
source ~/robotis_ws/install/setup.bash
```

### Launch the simulator:


```bash
# 1) Webots (opens GUI; make sure robot controller field is 'extern')
webots src/op3-webots/op3_webots_ros2/worelds/field.wbt
```
Webots should load, including the OP3 robot, waiting to connect to the external controller.  In the Webots console you should now see:

```bash
INFO: 'ROBOTIS_OP3_0' extern controller: Waiting for local or remote connection on port 1234 targeting robot named 'ROBOTIS_OP3_0'.
```
### Connect the external controller to webots

```bash

# 2) Extern controller (in another terminal)
export WEBOTS_CONTROLLER_URL=tcp://localhost:1234/ROBOTIS_OP3_0
ros2 run op3_webots_ros2 op3_extern_controller
```

Now you should see `INFO: 'ROBOTIS_OP3_0' extern controller: connected.` in the
Webots console.

### Launch the Robotis Manager for Simulation

```bash
ros2 launch op3_manager op3_simulation.launch.py
```
In the webots window you should see the robot move to the initial, resting position.
In a separate terminal window, enter a ros command to display the nodes running.

```bash
 ~/robotis_ws$ ros2 node list
/op3_action_module
/op3_base_module
/op3_direct_control_module
/op3_head_control_module
/op3_manager
/op3_online_walking_module
/op3_tuning_module
/op3_walking_module
/op3_webots_extern_controller
/open_cr_module
/robotis_controller

```

There are many topics running as the OP3 is very modular, so instead of listing all
topics, we will need to `grep` to filter the list.  We want to find the status topic
so that we can check what is happening as we run commands.

```bash
ros2 topic list | grep status
```

This shows that the topic is `/robotis/status`, we can use the `topic info` command
to find out more, this will show us how many nodes are publishing to it and how many
are subscribing to the topic.  It will also tell us what message type it is using.

```bash
❯ ros2 topic info /robotis/status
Type: robotis_controller_msgs/msg/StatusMsg
Publisher count: 8
Subscription count: 1
```

We can find out about the message type with 

```bash
❯ ros2 interface show robotis_controller_msgs/msg/StatusMsg

# Status Constants
uint8 STATUS_UNKNOWN = 0
uint8 STATUS_INFO = 1
uint8 STATUS_WARN = 2
uint8 STATUS_ERROR = 3

std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id
uint8 type
string module_name
string status_msg

```
Now that we have the details of the topic we want to watch, we can set a terminal to
echo all messages published to the topic.  You won't see anything until a message is
published to the topic.  When you see the message you will need the details of the
`StatusMsg` to decode it.

```bash
ros2 topic echo /robotis/status
```

### Use the action module to stand up

```bash
ros2 node list | grep action
ros2 topic list | grep action

/robotis/action/page_num
/robotis/action/start_action

❯ ros2 interface show op3_action_module_msgs/msg/StartAction

int32     page_num
string..  joint_name_array


ros2 topic pub --once /robotis/action/page_num std_msgs/msg/Int32 "data: 1"
ros2 topic pub --once /robotis/action/start_action op3_action_module_msgs/msg/StartAction "{page_number: 1}"


ros2 topic pub --once /robotis/action/start_action \
op3_action_module_msgs/msg/StartAction \
"{page_num: 1, joint_name_array: []}"

```

### Switch control module to walking

```bash
❯ ros2 topic pub --once /robotis/set_control_mode std_msgs/String \
  "data: walking_module"
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='walking_module')

```

## problems with python?

```bash
 ~/stride/robotis_ws ··········································· 08:46:31
❯ unset PYENV_VERSION
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v pyenv | paste -sd:)


 ~/stride/robotis_ws ··········································· 08:48:53
❯ which python                                                               which python3
which ros2
python --version

/home/karen/.local/bin/python
/home/karen/.local/bin/python3
/opt/ros/jazzy/bin/ros2
Python 3.12.3

# rebuild with correct version of python
 cd ~/robotis_ws
rm -rf build install log
colcon build --symlink-install

```
