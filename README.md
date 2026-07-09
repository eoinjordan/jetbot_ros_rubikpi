# JetBot ROS2 on Qualcomm 6490 suite of platforms e.g. Rubik Pi

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

Architecture overview: [docs/architecture.md](docs/architecture.md).

## Platform

Validated target:

- Ubuntu 24.04
- ROS 2 Jazzy
- Rubik Pi
- SparkFun JetBot AI Kit v2.0 or compatible JetBot-class chassis

Simulation in this repo still uses Gazebo Classic:

- `gazebo`
- `gazebo_ros`


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

Verify that the SparkFun Python driver is present before launching motors:

```bash
python3 -c "import qwiic; print(qwiic.__file__)"
```

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

Run Gazebo on a Linux development machine or WSL Ubuntu instance where Gazebo
Classic is actually available. Do not treat Gazebo as a Rubik Pi onboard
workflow.

Install simulation dependencies:

```bash
sudo apt install gazebo ros-jazzy-gazebo-ros
```

Build and source the overlay:

```bash
cd ~/jetbot_ros_rubikpi
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Launch the default world:

```bash
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

### Generic 2-wheel sim

The repo also includes a generic differential-drive model and matching worlds:

- model: `simple_diff_ros`
- worlds:
  - `dirt_path_simple_diff.world`
  - `dirt_path_curves_simple_diff.world`
  - `maze_simple_diff.world`
  - `maze_obstacles_simple_diff.world`

Generic robot launch:

```bash
ros2 launch jetbot_ros gazebo_world.launch.py \
  robot_model:=simple_diff_ros \
  robot_name:=simple_diff
```

Drive the generic robot:

```bash
ros2 topic pub -r 5 /simple_diff/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

For a strict simple-diff world, start Gazebo directly with one of the
`*_simple_diff.world` files and then run the repo spawner. See
[docs/architecture.md](docs/architecture.md).

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

That package publishes (default node namespace `/edgeimpulse_detector`):

- `vision_msgs/Detection2DArray` on `/edgeimpulse_detector/detections`
- an annotated `sensor_msgs/Image` on `/edgeimpulse_detector/debug_image` (when `publish_debug_image:=true`)
- `diagnostic_msgs/DiagnosticArray` on `/diagnostics` (FPS and latency)

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
sudo apt install -y ros-jazzy-vision-msgs ros-jazzy-diagnostic-msgs \
  ros-jazzy-v4l2-camera python3-opencv python3-numpy portaudio19-dev
sudo python3 -m pip install --break-system-packages -r ~/jetbot_ros_rubikpi/requirements-edge-impulse.txt
sudo python3 -m pip install --break-system-packages edge_impulse_linux pyaudio
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
  video_device:=/dev/video0
```

Important:

- `edgeimpulse_ros` subscribes to a `sensor_msgs/Image` topic; this launch starts a `v4l2_camera` driver that publishes it.
- Do not start a second camera driver on the same `/dev/video*` device.
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

Use `YUYV` on the validated Rubik Pi path:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_width:=640 \
  -p image_height:=480 \
  -p fps:=15.0 \
  -p pixel_format:="YUYV"
```

If `MJPG` is selected on this camera and image stack, `v4l2_camera` may abort
with an empty image encoding and a `cv_bridge::Exception`.

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
sudo apt install -y python3-serial python3-spidev
sudo python3 -m pip install --break-system-packages sparkfun-qwiic sparkfun-qwiic-scmd
```

If `scripts/rubikpi_provision.sh` stopped at `externally-managed-environment`,
pull the latest repo and rerun it. The old script attempted a system `pip`
upgrade, which Ubuntu 24.04 blocks.
