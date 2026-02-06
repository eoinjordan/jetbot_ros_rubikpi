# jetbot_ros
ROS2 nodes and Gazebo model for JetBot-class robots, validated for Rubik Pi (Ubuntu 24.04 / ROS 2 Jazzy).

<img width="901" height="712" alt="image" src="https://github.com/user-attachments/assets/3b368e39-3d75-4b1e-b3b4-19c854777292" />


> note:  if you want to use ROS Melodic, see the [`melodic`](https://github.com/dusty-nv/jetbot_ros/tree/melodic) branch

### SparkFun kit resources

- SparkFun JetBot AI Kit v2.0 assembly guide (previous version): https://learn.sparkfun.com/tutorials/assembly-guide-for-sparkfun-jetbot-ai-kit-v20/resources-and-going-further
- SparkFun Qwiic Motor Driver hookup guide: https://learn.sparkfun.com/tutorials/sparkfun-qwiic-motor-driver-hookup-guide
- SparkFun Qwiic Micro OLED hookup guide: https://learn.sparkfun.com/tutorials/sparkfun-qwiic-micro-oled-hookup-guide
- SparkFun Qwiic pHAT hookup guide: https://learn.sparkfun.com/tutorials/sparkfun-qwiic-phat-hookup-guide

### Qwiic (I2C) setup checklist

```bash
sudo apt install -y i2c-tools python3-smbus
sudo usermod -aG i2c $USER
# log out/in after this, then:
i2cdetect -y 1
```
### Run JetBot (SparkFun JetBot AI Kit v2.0 on Rubik Pi / Ubuntu 24.04)

These instructions assume the SparkFun JetBot AI Kit (Qwiic pHAT + Qwiic Motor Driver + Qwiic Micro OLED) on Rubik Pi running Ubuntu 24.04 with ROS 2 Jazzy.

``` bash
ros2 launch jetbot_ros jetbot_cpu.launch.py motor_controller:=motors_sparkfun
```

Make sure the v4l2 camera node is installed (Jazzy on Ubuntu 24.04):

``` bash
sudo apt install ros-jazzy-v4l2-camera
```

If you are on Rubik Pi, you can provision the full stack using:

``` bash
sudo bash scripts/rubikpi_provision.sh
```

After provisioning, create a venv and build the ROS overlay (required for `ros2 launch`):

```bash
cd ~/jetbot_ros_rubikpi
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install torch==2.2.2 --extra-index-url https://download.pytorch.org/whl/cpu
pip install -e .

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If you see `package 'jetbot_ros' not found`, it means the overlay is not sourced. Run:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Run the smoke checks to validate hardware and ROS connectivity:

```bash
cd ~/jetbot_ros_rubikpi
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
./scripts/smoke_check.sh
```

### USB webcam troubleshooting (Logitech StreamCam and low-cost USB cams)

If you see `Failed mapping device memory`, run the camera node with MJPG and a lower resolution:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
	-p video_device:=/dev/video0 \
	-p image_width:=320 -p image_height:=240 -p fps:=15.0 \
	-p pixel_format:="MJPG"
```

Note: `pixel_format` must be a 4‑character code (FOURCC). Use `MJPG` (not `mjpeg`).

Note: ROS 2 nodes launched with `ros2` use system Python by default. For `motors_sparkfun`, ensure Qwiic is installed into system Python (the provisioner does this). If you skipped provisioning, run:

```bash
sudo python3 -m pip install --break-system-packages sparkfun-qwiic pyserial spidev
```


Other motor controller options:

``` bash
ros2 launch jetbot_ros jetbot_cpu.launch.py motor_controller:=motors_waveshare
```

If you use `motors_waveshare`, install the MotorHAT dependency in your venv:

```bash
source .venv/bin/activate
pip install Adafruit-MotorHAT
```

See the [`Launch Gazebo`](#launch-gazebo) section below to run the simulator.

### Launch Gazebo

Install Gazebo dependencies (only needed for simulation):

``` bash
sudo apt install gazebo ros-jazzy-gazebo-ros
```

Build and source the overlay before launching:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

``` bash
ros2 launch jetbot_ros gazebo_world.launch.py
```

### Test Teleop

``` bash
ros2 launch jetbot_ros teleop_keyboard.launch.py
```

The keyboard controls are as follows:

```
w/x:  increase/decrease linear velocity
a/d:  increase/decrease angular velocity

space key, s:  force stop
```

Press Ctrl+C to quit.

### Motor test (SparkFun Qwiic)

Terminal 1:

```bash
ros2 launch jetbot_ros jetbot_cpu.launch.py motor_controller:=motors_sparkfun
```

Terminal 2:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.05}, angular: {z: 0.0}}"
```

### Data Collection

``` bash
ros2 launch jetbot_ros data_collection.launch.py
```

It's recommended to view the camera feed in Gazebo by going to `Window -> Topic Visualization -> gazebo.msgs.ImageStamped` and selecting the `/gazebo/default/jetbot/camera_link/camera/image` topic.

Then drive the robot and press the `C` key to capture an image.  Then annotate that image in the pop-up window by clicking the center point of the path.  Repeat this all the way around the track.  It's important to also collect data of when the robot gets off-course (i.e. near the edges of the track, or completely off the track).  This way, the JetBot will know how to get back on track.

Press Ctrl+C when you're done collecting data to quit.

### Train Navigation Model

Run this locally, substituting the path of the dataset that you collected (by default, it will be in a timestamped folder under `~/jetbot_ros_rubikpi/data/datasets/`)

``` bash
cd ~/jetbot_ros_rubikpi/jetbot_ros/dnn
python3 train.py --data ~/jetbot_ros_rubikpi/data/datasets/20211018-160950/
```

### Run Navigation Model

After the model has finished training, run the command below to have the JetBot navigate autonomously around the track.  Substitute the path to your model below:

``` bash
ros2 launch jetbot_ros nav_model.launch.py model:=~/jetbot_ros_rubikpi/data/models/202106282129/model_best.pth
```

> note:  to reset the position of the robot in the Gazebo environment, press `Ctrl+R`

<a href="https://youtu.be/gok9pvUzZeY" target="_blank"><img src=https://github.com/dusty-nv/jetbot_ros/raw/dev/docs/images/jetbot_gazebo_sim_video.jpg width="750"></a>


