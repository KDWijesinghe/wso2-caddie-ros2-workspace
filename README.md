# WSO2 Caddie ROS 2 Simulation

ROS 2 Jazzy simulation workspace for an autonomous robotic caddie based on the
Unitree Go2 Edu MVP proposal. The stack uses Gazebo Sim, SLAM Toolbox, Nav2,
RGB-D golf-ball perception, VLM-ready scene context, voice/text commands, and a
main autonomy node that orchestrates mapping, ball retrieval, return-home, and
shot analytics.

The proposal PDF is image-based, but the relevant requirements are reflected in
this workspace:

- Unitree Go2 Edu quadruped platform for terrain agility.
- Autonomous course navigation using ROS 2, SLAM, and Nav2.
- RGB-D vision with YOLO plus VLM-style scene reasoning for golf-ball tracking.
- LLM/voice command interaction.
- Golf equipment logistics, ball retrieval, and shot analytics.
- MVP payload awareness for Go2 Edu; the simulated bag rack represents a
  research payload, not a production loadout.

## Package Layout

```text
src/
  go2_description/      Official Unitree Go2 URDF meshes/assets, ROS 2 packaged
  unitree_api/          Official Unitree ROS 2 API message definitions
  unitree_go/           Official Unitree Go2 ROS 2 message definitions
  caddie_unitree_official/
                        Official Unitree MuJoCo Go2 model/assets for reference
  caddie_description/   Go2 caddie URDF/Xacro and controller config
  caddie_gazebo/        Gazebo Sim course world and ROS-GZ bridge launch
  caddie_navigation/    SLAM Toolbox, Nav2 params, RViz config
  caddie_perception/    YOLO/OpenCV golf-ball detector and VLM context node
  caddie_interaction/   Vosk-style voice node and conversational router
  caddie_core/          Main autonomous caddie orchestration node
  caddie_control/       Optional velocity limiter / Unitree SDK adapter point
  caddie_bringup/       One-command simulation bringup
```

## Official Unitree Integration

This workspace now uses official Unitree project assets where they fit ROS 2
Jazzy and Gazebo Sim:

- `src/go2_description` vendors the official Go2 URDF, DAE meshes, and control
  config from Unitree's ROS repository:
  <https://github.com/unitreerobotics/unitree_ros>
- `src/unitree_api` and `src/unitree_go` vendor the official Unitree ROS 2
  message packages used by the SDK2/SportMode API path:
  <https://github.com/unitreerobotics/unitree_ros2>
- `src/caddie_unitree_official/mujoco/go2` keeps Unitree's official MuJoCo Go2
  XML and terrain scenes for physics/reference testing outside Gazebo Sim:
  <https://github.com/unitreerobotics/unitree_mujoco>
- `src/caddie_description/urdf/go2_caddie_official.urdf.xacro` is the default
  simulation model. It extends the official Go2 body with caddie payload,
  RGB-D camera, lidar, IMU, and Gazebo Sim control/sensor plugins.

The official Unitree Gazebo/ROS examples target different simulator/runtime
paths, so this project keeps Gazebo Sim as the main ROS 2 test environment and
adds an SDK-compatible bridge point for the real robot or Unitree middleware.

## Install Dependencies

```bash
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
```

Optional Python packages:

```bash
pip3 install --break-system-packages opencv-python numpy ultralytics vosk sounddevice
```

`ultralytics` is only needed for YOLO. If `yolo_model` is empty or the package
is missing, the detector uses the OpenCV white/circular golf-ball fallback.
`vosk` and `sounddevice` are only needed for microphone input; text commands
work without them.

## Build

```bash
cd /media/nimsika/WindowsData/ros2/WSO2-caddie-project
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run The Full Simulation

```bash
ros2 launch caddie_bringup caddie_sim.launch.py
```

Useful launch options:

```bash
ros2 launch caddie_bringup caddie_sim.launch.py gui:=false
ros2 launch caddie_bringup caddie_sim.launch.py use_rviz:=false
ros2 launch caddie_bringup caddie_sim.launch.py detector_backend:=opencv
ros2 launch caddie_bringup caddie_sim.launch.py yolo_model:=/path/to/golf_ball_yolo.pt
ros2 launch caddie_bringup caddie_sim.launch.py use_voice:=true
ros2 launch caddie_bringup caddie_sim.launch.py use_unitree_sport_bridge:=true
ros2 launch caddie_bringup caddie_sim.launch.py use_leg_animation:=false
```

## Test Commands

The voice node always supports text fallback:

```bash
ros2 topic pub --once /caddie/text_command std_msgs/msg/String "{data: 'start mapping'}"
ros2 topic pub --once /caddie/text_command std_msgs/msg/String "{data: 'retrieve nearest ball'}"
ros2 topic pub --once /caddie/text_command std_msgs/msg/String "{data: 'list balls'}"
ros2 topic pub --once /caddie/text_command std_msgs/msg/String "{data: 'analyze shot'}"
ros2 topic pub --once /caddie/text_command std_msgs/msg/String "{data: 'return home'}"
ros2 topic pub --once /caddie/text_command std_msgs/msg/String "{data: 'stop'}"
```

The conversational router accepts free-form text:

```bash
ros2 topic pub --once /caddie/conversation_text std_msgs/msg/String "{data: 'please find the closest lost golf ball'}"
ros2 topic pub --once /caddie/conversation_text std_msgs/msg/String "{data: 'take me back to the tee box'}"
```

Useful status topics:

```bash
ros2 topic echo /caddie/status
ros2 topic echo /caddie/ball_detections
ros2 topic echo /go2/gait_status
```

## Notes On The Go2 Simulation Model

The default URDF/Xacro is based on the official Unitree Go2 description. For
robust Nav2 simulation it uses hidden tiny drive wheels under the body and
Gazebo's diff-drive system plugin to consume `/cmd_vel` and publish `/odom`.
By default, the visible Go2 legs are animated through `gz_ros2_control` while
the hidden wheels remain responsible for stable odometry and base movement.
The animated leg links are visual-only in Gazebo, so their moving feet do not
strike the ground and tip the robot over.

The gait animator subscribes to `/cmd_vel`, `/odom`, and `/imu`. It changes
stride length, step frequency, stance width, and foot lift for the course
surface under the robot:

- fairway: normal trot
- green: short, gentle steps
- tee: cautious startup stance
- rough: shorter stride with extra lift
- sand bunkers: slower high-clearance steps
- mound/slope: wider stance with pitch/roll compensation

To run without visible leg movement:

```bash
ros2 launch caddie_bringup caddie_sim.launch.py use_leg_animation:=false
```

The gait is still a visualizer, not Unitree's production locomotion controller.
Tune it live after launch if you want a slower or more expressive walk:

```bash
ros2 param set /go2_gait_animator max_step_frequency 0.55
ros2 param set /go2_gait_animator max_thigh_swing 0.045
ros2 param set /go2_gait_animator max_calf_swing 0.032
ros2 param set /go2_gait_animator trajectory_time 0.70
```

The optional velocity limiter remains useful when feeding commands from Nav2 or
teleop:

```bash
ros2 run caddie_control go2_velocity_limiter \
  --ros-args -p input_topic:=/cmd_vel_raw -p output_topic:=/cmd_vel
```

The optional Unitree SportMode bridge publishes official `unitree_api/Request`
messages to `/api/sport/request`:

```bash
ros2 run caddie_control unitree_sportmode_bridge
```

Use that bridge only when the Unitree ROS 2 middleware or robot-side SDK agent
is available. In Gazebo Sim, leave `use_unitree_sport_bridge:=false` unless you
are testing the message flow.
