WSO2 Caddie ROS 2 Simulation

ROS 2 Jazzy simulation workspace for an autonomous robotic caddie based on the Unitree Go2 Edu MVP proposal. The stack supports twin simulation pipelines: standard Gazebo Sim with visual leg animations, and a high-fidelity MuJoCo Sim + Zenoh Middleware environment featuring custom holonomic trot kinematics.

The proposal PDF is image-based, but the relevant requirements are reflected in this workspace:

Unitree Go2 Edu quadruped platform for terrain agility.

Autonomous course navigation using ROS 2, SLAM, and Nav2.

RGB-D vision with YOLO plus VLM-style scene reasoning for golf-ball tracking.

LLM/voice command interaction.

Golf equipment logistics, ball retrieval, and shot analytics.

MVP payload awareness for Go2 Edu; the simulated bag rack represents a research payload, not a production loadout.

Package Layout

src/
  go2_description/      Official Unitree Go2 URDF meshes/assets, ROS 2 packaged
  unitree_api/          Official Unitree ROS 2 API message definitions
  unitree_go/           Official Unitree Go2 ROS 2 message definitions
  caddie_unitree_official/
                        Official Unitree MuJoCo Go2 model/assets & custom Zenoh controller
  caddie_description/   Go2 caddie URDF/Xacro and controller config
  caddie_gazebo/        Gazebo Sim course world and ROS-GZ bridge launch
  caddie_navigation/    SLAM Toolbox, Nav2 params, RViz config
  caddie_perception/    YOLO/OpenCV golf-ball detector and VLM context node
  caddie_interaction/   Vosk-style voice node and conversational router
  caddie_core/          Main autonomous caddie orchestration node
  caddie_control/       Optional velocity limiter / Unitree SDK adapter point
  caddie_bringup/       One-command simulation bringup


Official Unitree Integration

This workspace uses official Unitree project assets where they fit ROS 2 Jazzy, Gazebo Sim, and MuJoCo:

src/go2_description vendors the official Go2 URDF, DAE meshes, and control config from Unitree's ROS repository: https://github.com/unitreerobotics/unitree_ros

src/unitree_api and src/unitree_go vendor the official Unitree ROS 2 message packages used by the SDK2/SportMode API path: https://github.com/unitreerobotics/unitree_ros2

src/caddie_unitree_official/mujoco/go2 keeps Unitree's official MuJoCo Go2 XML and terrain scenes for high-fidelity physics, extended with a standalone Zenoh Python controller bridge (run_dog.py): https://github.com/unitreerobotics/unitree_mujoco

src/caddie_description/urdf/go2_caddie_official.urdf.xacro extends the official Go2 body with caddie payload, RGB-D camera, lidar, IMU, and Gazebo Sim control/sensor plugins.

Install Dependencies

1. Standard ROS 2 Jazzy & Gazebo Prerequisites

sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-xacro \
  ros-jazzy-tf2-geometry-msgs \
  python3-colcon-common-extensions \
  python3-rosdep


2. Zenoh Middleware & MuJoCo Requirements

# Install ROS 2 Zenoh RMW backend package
sudo apt install -y ros-jazzy-rmw-zenoh-cpp

# Install Python requirements for standalone MuJoCo tracking
pip3 install mujoco mujoco-python-viewer eclipse-zenoh --break-system-packages opencv-python numpy ultralytics vosk sounddevice


Note: ultralytics is only needed for YOLO. If yolo_model is empty or the package is missing, the detector uses the OpenCV white/circular golf-ball fallback. vosk and sounddevice are only needed for microphone input; text commands work without them.

Build

cd /media/nimsika/WindowsData/ros2/WSO2-caddie-project
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash


Running Pipeline 1: Full Gazebo Simulation

ros2 launch caddie_bringup caddie_sim.launch.py


Useful Launch Options:

ros2 launch caddie_bringup caddie_sim.launch.py gui:=false
ros2 launch caddie_bringup caddie_sim.launch.py use_rviz:=false
ros2 launch caddie_bringup caddie_sim.launch.py detector_backend:=opencv
ros2 launch caddie_bringup caddie_sim.launch.py yolo_model:=/path/to/golf_ball_yolo.pt
ros2 launch caddie_bringup caddie_sim.launch.py use_voice:=true
ros2 launch caddie_bringup caddie_sim.launch.py use_unitree_sport_bridge:=true
ros2 launch caddie_bringup caddie_sim.launch.py use_leg_animation:=false


Running Pipeline 2: High-Fidelity MuJoCo + Zenoh Simulation

This mode bypasses the standard heavy simulation nodes to track precise leg-ground contacts, executing via custom Omnidirectional Sine-Wave Trot Kinematics mapped directly over standard ROS 2 /cmd_vel inputs bridged through Zenoh.

Step 1: Fire up the background Zenoh Router

ros2 run rmw_zenoh_cpp rmw_zenohd


Step 2: Spin up the MuJoCo Walk Bridge Controller

cd /media/nimsika/WindowsData/ros2/WSO2-caddie-project
source install/setup.bash
cd src/caddie_unitree_official/mujoco/go2
python3 run_dog.py


The Go2 platform will spawn in a sturdy, stable horizontal stance ($Kp=160.0, Kd=8.0$) balancing its internal floating-base frames.

Step 3: Publish Omnidirectional Velocities (Test Commands)

Open another terminal window, explicitly set your RMW transport layer configuration, and fire commands:

Walk Forward / Backward:

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}}"


(Backward paths use absolute abs(vx) scaling to protect calf clearance constraints).

Lateral Side-Steps (Crab-Walking):

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.2, z: 0.0}}"


Pivot Turning (Fixed Pivot Yaw Kinematics):

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}"


(Coordinated via our custom cross-mapping: Left flank joints actuate sideways along the Y-axis, while Right flank segments stroke linearly forward along the X-axis).

Autonomous Task Test Commands

The voice node always supports text fallback:

ros2 topic pub --once /caddie/text_command std_msgs/msg/String "{data: 'start mapping'}"
ros2 topic pub --once /caddie/text_command std_msgs/msg/String "{data: 'retrieve nearest ball'}"
ros2 topic pub --once /caddie/text_command std_msgs/msg/String "{data: 'list balls'}"
ros2 topic pub --once /caddie/text_command std_msgs/msg/String "{data: 'analyze shot'}"
ros2 topic pub --once /caddie/text_command std_msgs/msg/String "{data: 'return home'}"
ros2 topic pub --once /caddie/text_command std_msgs/msg/String "{data: 'stop'}"


The conversational router accepts free-form text:

ros2 topic pub --once /caddie/conversation_text std_msgs/msg/String "{data: 'please find the closest lost golf ball'}"
ros2 topic pub --once /caddie/conversation_text std_msgs/msg/String "{data: 'take me back to the tee box'}"


Useful Status Topics:

ros2 topic echo /caddie/status
ros2 topic echo /caddie/ball_detections
ros2 topic echo /go2/gait_status


Notes On The Go2 Simulation Model

The default URDF/Xacro is based on the official Unitree Go2 description. For robust Nav2 simulation it uses hidden tiny drive wheels under the body and Gazebo's diff-drive system plugin to consume /cmd_vel and publish /odom.

By default, the visible Go2 legs are animated through gz_ros2_control while the hidden wheels remain responsible for stable odometry and base movement. The animated leg links are visual-only in Gazebo, so their moving feet do not strike the ground and tip the robot over.

The gait animator subscribes to /cmd_vel, /odom, and /imu. It changes stride length, step frequency, stance width, and foot lift for the course surface under the robot:

fairway: normal trot

green: short, gentle steps

tee: cautious startup stance

rough: shorter stride with extra lift

sand bunkers: slower high-clearance steps

mound/slope: wider stance with pitch/roll compensation

To run without visible leg movement:

ros2 launch caddie_bringup caddie_sim.launch.py use_leg_animation:=false


The gait is still a visualizer, not Unitree's production locomotion controller. Tune it live after launch if you want a slower or more expressive walk:

ros2 param set /go2_gait_animator max_step_frequency 0.55
ros2 param set /go2_gait_animator max_thigh_swing 0.045
ros2 param set /go2_gait_animator max_calf_swing 0.032
ros2 param set /go2_gait_animator trajectory_time 0.70


The optional velocity limiter remains useful when feeding commands from Nav2 or teleop:

ros2 run caddie_control go2_velocity_limiter --ros-args -p input_topic:=/cmd_vel_raw -p output_topic:=/cmd_vel


The optional Unitree SportMode bridge publishes official unitree_api/Request messages to /api/sport/request:

ros2 run caddie_control unitree_sportmode_bridge


Use that bridge only when the Unitree ROS 2 middleware or robot-side SDK agent is available. In Gazebo Sim, leave use_unitree_sport_bridge:=false unless you are testing the message flow.


ros2 topic pub /caddie/text_command std_msgs/msg/String "data: 'hit'" --once


publisher: beginning loop
publishing #1: std_msgs.msg.String(data='hit')