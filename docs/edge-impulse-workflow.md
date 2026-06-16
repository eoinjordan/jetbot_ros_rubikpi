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

This path lets `edgeimpulse_ros` open the camera device directly. Do not run the
regular `v4l2_camera` node on the same device at the same time.

```bash
cd ~/jetbot_edgeimpulse_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch jetbot_ros jetbot_edge_impulse.launch.py \
  motor_controller:=motors_sparkfun \
  model_path:=/home/$USER/modelfile.eim \
  camera:=0
```

Published topics:

- `/edgeimpulse/detections` (`vision_msgs/Detection2DArray`)
- `/edgeimpulse/timing` (`std_msgs/String`)
- `/edgeimpulse/count` (`std_msgs/Int32`)

## Gazebo

Gazebo simulation stays separate from `edgeimpulse_ros` for now. The current
`edgeimpulse_ros` package reads frames directly from a V4L2 camera index, not a
ROS image topic, so it is a real-camera workflow rather than a Gazebo camera
workflow.

On Rubik Pi specifically, treat Gazebo as an off-board development workflow.
Some Ubuntu images for Rubik Pi do not provide Gazebo Classic packages, so the
Rubik Pi provisioner installs the real-hardware stack first and skips Gazebo
when those packages are unavailable.
