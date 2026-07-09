# JetBot Edge Impulse Workflow

This repository uses two packages together:

- `jetbot_ros` for motors, teleop, Gazebo, and JetBot-specific ROS nodes
- `edgeimpulse_ros` for camera-based `.eim` object detection and ROS
  `vision_msgs/Detection2DArray` output

## Workspace Layout

Create one ROS workspace containing both repositories:

```bash
cd ~/jetbot_ros_rubikpi
bash scripts/setup_edge_impulse_workspace.sh \
  ~/jetbot_edgeimpulse_ws \
  ~/jetbot_ros_rubikpi \
  ~/edgeimpulse-ros
```

Then build it:

```bash
cd ~/jetbot_edgeimpulse_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Install Edge Impulse Runtime

```bash
sudo apt update
sudo apt install -y ros-jazzy-vision-msgs python3-opencv
sudo python3 -m pip install --break-system-packages -r ~/jetbot_ros_rubikpi/requirements-edge-impulse.txt
sudo python3 -m pip install --break-system-packages edge_impulse_linux
```

Download the `.eim` model:

```bash
edge-impulse-linux-runner --download modelfile.eim
```

## Run On Real JetBot Hardware

`edgeimpulse_ros` no longer opens the camera itself; it subscribes to a
`sensor_msgs/Image` topic. This launch starts a `v4l2_camera` driver that
publishes to `image_topic` (default `/jetbot/camera/image_raw`) and points the
detector at it, so do not start a second camera driver on the same device.

```bash
cd ~/jetbot_edgeimpulse_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch jetbot_ros jetbot_edge_impulse.launch.py \
  motor_controller:=motors_sparkfun \
  model_path:=/home/$USER/modelfile.eim \
  video_device:=/dev/video0
```

Published topics (default node namespace `/edgeimpulse_detector`):

- `/edgeimpulse_detector/detections` (`vision_msgs/Detection2DArray`)
- `/edgeimpulse_detector/debug_image` (`sensor_msgs/Image`, when `publish_debug_image:=true`)
- `/diagnostics` (`diagnostic_msgs/DiagnosticArray`, FPS and latency)

## Gazebo

Gazebo simulation stays separate from `edgeimpulse_ros` for now. `edgeimpulse_ros`
subscribes to a `sensor_msgs/Image` topic, so in principle it can consume a
Gazebo camera stream, but this repo validates it only against the real V4L2
camera path.

On Rubik Pi specifically, treat Gazebo as an off-board development workflow.
Some Ubuntu images for Rubik Pi do not provide Gazebo Classic packages, so the
Rubik Pi provisioner installs the real-hardware stack first and skips Gazebo
when those packages are unavailable.

For the generic 2-wheel Gazebo path and architecture split, see
[architecture.md](architecture.md).
