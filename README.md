# JetBot ROS on Rubik Pi

ROS 2 Jazzy nodes, Gazebo simulation assets, and Edge Impulse integration for a
JetBot-class robot on Rubik Pi and Ubuntu 24.04.

This repository covers four concrete workflows:

- Real JetBot bringup on Rubik Pi with SparkFun or Waveshare motor control
- Gazebo simulation for navigation and dataset generation
- Local navigation model training and replay
- Edge Impulse camera detection using the separate `edgeimpulse_ros` package

<img width="901" height="712" alt="robots" src="https://github.com/user-attachments/assets/f6f2c591-a1bd-448d-b768-a183fa6713ea" />

## What Is In This Repo

```text
jetbot_ros/                Python ROS 2 nodes for motors, teleop, nav, data collection
launch/                    Real hardware, teleop, Gazebo, and Edge Impulse launch files
gazebo/models/             JetBot and track models
gazebo/worlds/             Dirt path and maze worlds
docs/                      Setup and workflow notes
scripts/                   Provisioning and workspace helpers
data/                      Local datasets and trained models
```

## Platform

Validated target:

- Ubuntu 24.04
- ROS 2 Jazzy
- Rubik Pi
- SparkFun JetBot AI Kit v2.0 or compatible JetBot-class chassis

Simulation in this repo still uses Gazebo Classic:

- `gazebo`
- `gazebo_ros`

If you want Gazebo Harmonic or modern `ros_gz_*` tooling, that is a separate
migration.

## Hardware Bringup

### SparkFun kit resources

- SparkFun JetBot AI Kit v2.0 assembly guide:
  https://learn.sparkfun.com/tutorials/assembly-guide-for-sparkfun-jetbot-ai-kit-v20/resources-and-going-further
- SparkFun Qwiic Motor Driver hookup guide:
  https://learn.sparkfun.com/tutorials/sparkfun-qwiic-motor-driver-hookup-guide
- SparkFun Qwiic Micro OLED hookup guide:
  https://learn.sparkfun.com/tutorials/sparkfun-qwiic-micro-oled-hookup-guide
- SparkFun Qwiic pHAT hookup guide:
  https://learn.sparkfun.com/tutorials/sparkfun-qwiic-phat-hookup-guide

### Qwiic / I2C checklist

```bash
sudo apt install -y i2c-tools python3-smbus
sudo usermod -aG i2c $USER
# log out and back in, then:
i2cdetect -y 1
```

### Rubik Pi provisioning

```bash
cd ~/jetbot_ros_rubikpi
sudo bash scripts/rubikpi_provision.sh
```

That installs:

- ROS 2 Jazzy base packages
- `v4l2_camera`
- `vision_msgs`
- OpenCV
- Qwiic / motor / OLED Python dependencies

On some Rubik Pi Ubuntu images, Gazebo Classic packages are not available.
The provisioner now skips Gazebo in that case so real hardware bringup still
works.

## Build

This package can be built standalone:

```bash
cd ~/jetbot_ros_rubikpi
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If `ros2 launch jetbot_ros ...` says the package is not found:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Run Real JetBot

### SparkFun motor controller

```bash
ros2 launch jetbot_ros jetbot_cpu.launch.py motor_controller:=motors_sparkfun
```

### Waveshare motor controller

```bash
ros2 launch jetbot_ros jetbot_cpu.launch.py motor_controller:=motors_waveshare
```

If you use `motors_waveshare`, install:

```bash
pip install Adafruit-MotorHAT
```

### Camera package

Install the ROS camera node:

```bash
sudo apt install ros-jazzy-v4l2-camera
```

The default launch remaps camera output to:

```text
/jetbot/camera/image_raw
```

### Motor smoke test

Terminal 1:

```bash
ros2 launch jetbot_ros jetbot_cpu.launch.py motor_controller:=motors_sparkfun
```

Terminal 2:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.05}, angular: {z: 0.0}}"
```

## Teleop

```bash
ros2 launch jetbot_ros teleop_keyboard.launch.py
```

Keys:

```text
w/x  increase/decrease linear velocity
a/d  increase/decrease angular velocity
space or s  force stop
```

## Gazebo Simulation

Install simulation dependencies on a Linux development machine where Gazebo
Classic is actually available:

```bash
sudo apt install gazebo ros-jazzy-gazebo-ros
```

Build and source the overlay, then launch:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch jetbot_ros gazebo_world.launch.py
```

Navigation simulation:

```bash
ros2 launch jetbot_ros gazebo_nav.launch.py
```

Worlds live under:

```text
gazebo/worlds/
```

Notable worlds:

- `dirt_path_curves.world`
- `maze.world`
- `maze_obstacles.world`

### Sim to real checklist

1. Verify Gazebo:

```bash
ros2 launch jetbot_ros gazebo_world.launch.py
```

2. Verify real hardware:

```bash
ros2 launch jetbot_ros jetbot_cpu.launch.py motor_controller:=motors_sparkfun
```

3. Publish a short drive command:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

## Data Collection

```bash
ros2 launch jetbot_ros data_collection.launch.py
```

In Gazebo, view the camera feed from:

```text
/gazebo/default/jetbot/camera_link/camera/image
```

Drive the robot and press `C` to capture labeled path data.

Datasets are stored under:

```text
data/datasets/
```

## Train Navigation Model

```bash
cd ~/jetbot_ros_rubikpi/jetbot_ros/dnn
python3 train.py --data ~/jetbot_ros_rubikpi/data/datasets/<timestamp>/
```

## Run Navigation Model

```bash
ros2 launch jetbot_ros nav_model.launch.py model:=~/jetbot_ros_rubikpi/data/models/<timestamp>/model_best.pth
```

## Edge Impulse

The recommended camera-detection workflow now uses the separate local package:

```text
C:\Users\Eoin\git\edgeimpulse-ros
```

That package publishes:

- `vision_msgs/Detection2DArray` on `/edgeimpulse/detections`
- timing metadata on `/edgeimpulse/timing`
- detection count on `/edgeimpulse/count`

### Build a shared workspace with `edgeimpulse_ros`

Because `edgeimpulse_ros` is a separate package, build both repos in one
workspace:

```bash
cd ~/jetbot_ros_rubikpi
bash scripts/setup_edge_impulse_workspace.sh \
  ~/jetbot_edgeimpulse_ws \
  ~/jetbot_ros_rubikpi \
  ~/edgeimpulse-ros

cd ~/jetbot_edgeimpulse_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Install Edge Impulse runtime

```bash
sudo apt install -y ros-jazzy-vision-msgs python3-opencv
sudo python3 -m pip install --break-system-packages -r ~/jetbot_ros_rubikpi/requirements-edge-impulse.txt
sudo python3 -m pip install --break-system-packages edge_impulse_linux
```

Download the `.eim` model:

```bash
edge-impulse-linux-runner --download modelfile.eim
```

### Real hardware detection workflow

Launch JetBot motor control plus the Edge Impulse detector:

```bash
cd ~/jetbot_edgeimpulse_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch jetbot_ros jetbot_edge_impulse.launch.py \
  motor_controller:=motors_sparkfun \
  model_path:=/home/$USER/modelfile.eim \
  camera:=0
```

Important:

- `edgeimpulse_ros` opens the camera device directly.
- Do not run the normal `v4l2_camera` node on the same camera at the same time.
- This is the supported real-camera workflow.

### Motor health workflow

The motor-health path remains local to this repo.

Inference node:

```bash
ros2 run jetbot_ros ei_motor_health_node --ros-args \
  -p model_path:=modelfile.eim \
  -p window_size:=300
```

CSV upload helper:

```bash
export EDGE_IMPULSE_API_KEY=your_api_key
export EDGE_IMPULSE_HMAC_KEY=your_hmac_key
ros2 run jetbot_ros ei_motor_health_collect -- \
  --input motor_health.csv \
  --interval-ms 16 \
  --label motor_health
```

More detail:

- [docs/edge-impulse-workflow.md](docs/edge-impulse-workflow.md)

## Troubleshooting

### `package 'jetbot_ros' not found`

Source the overlay:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### USB webcam `Failed mapping device memory`

Use MJPG and lower resolution:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_width:=320 \
  -p image_height:=240 \
  -p fps:=15.0 \
  -p pixel_format:="MJPG"
```

### USB webcam `Device or resource busy`

Another process is using the camera:

```bash
ros2 node list
ros2 node kill /v4l2_camera
```

### ROS + system Python note

ROS 2 launch typically uses system Python. If you skipped the provisioner and a
hardware node fails to import Qwiic libraries, install them into system Python:

```bash
sudo python3 -m pip install --break-system-packages sparkfun-qwiic pyserial spidev
```
