# jetbot_ros
ROS2 nodes and Gazebo model for JetBot-class robots, validated for Rubik Pi (hardware-accelerated workflows) and Raspberry Pi (CPU-only workflows).

<img width="901" height="712" alt="image" src="https://github.com/user-attachments/assets/3b368e39-3d75-4b1e-b3b4-19c854777292" />


> note:  if you want to use ROS Melodic, see the [`melodic`](https://github.com/dusty-nv/jetbot_ros/tree/melodic) branch

### Start the JetBot ROS2 Jazzy container (optional, NVIDIA Jetson legacy)

``` bash
git clone https://github.com/dusty-nv/jetbot_ros
cd jetbot_ros
docker/run.sh
```
 
### Run JetBot (Rubik Pi / Raspberry Pi)

For Rubik Pi or Raspberry Pi on Ubuntu 24.04, use ROS 2 Jazzy and the CPU camera pipeline:

``` bash
ros2 launch jetbot_ros jetbot_cpu.launch.py motor_controller:=motors_waveshare
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
cd ~/jetbot_ros
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install torch==2.2.2 --extra-index-url https://download.pytorch.org/whl/cpu
pip install -e .

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run the smoke checks to validate hardware and ROS connectivity:

```bash
cd ~/jetbot_ros
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
./scripts/smoke_check.sh
```


Other motor controller options:

``` bash
ros2 launch jetbot_ros jetbot_cpu.launch.py motor_controller:=motors_sparkfun
```

If you are on a legacy NVIDIA Jetson setup, you can still use:

``` bash
ros2 launch jetbot_ros jetbot_nvidia.launch.py
```

Otherwise, see the [`Launch Gazebo`](#launch-gazebo) section below to run the simulator.

### Launch Gazebo

``` bash
ros2 launch jetbot_ros gazebo_world.launch.py
```

> **Note:** Gazebo simulation is currently not included in the Docker build due to compatibility issues on M1 Macs. This will be revisited for future updates.

Then to run the following commands, launch a new terminal session into the container:

``` bash
sudo docker exec -it jetbot_ros /bin/bash
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

### Data Collection

``` bash
ros2 launch jetbot_ros data_collection.launch.py
```

It's recommended to view the camera feed in Gazebo by going to `Window -> Topic Visualization -> gazebo.msgs.ImageStamped` and selecting the `/gazebo/default/jetbot/camera_link/camera/image` topic.

Then drive the robot and press the `C` key to capture an image.  Then annotate that image in the pop-up window by clicking the center point of the path.  Repeat this all the way around the track.  It's important to also collect data of when the robot gets off-course (i.e. near the edges of the track, or completely off the track).  This way, the JetBot will know how to get back on track.

Press Ctrl+C when you're done collecting data to quit.

### Train Navigation Model

Run this from inside the container, substituting the path of the dataset that you collected (by default, it will be in a timestamped folder under `/workspace/src/jetbot_ros/data/datasets/`)

``` bash
cd /workspace/src/jetbot_ros/jetbot_ros/dnn
python3 train.py --data /workspace/src/jetbot_ros/data/datasets/20211018-160950/
```

### Run Navigation Model

After the model has finished training, run the command below to have the JetBot navigate autonomously around the track.  Substitute the path to your model below:

``` bash
ros2 launch jetbot_ros nav_model.launch.py model:=/workspace/src/jetbot_ros/data/models/202106282129/model_best.pth
```

> note:  to reset the position of the robot in the Gazebo environment, press `Ctrl+R`

<a href="https://youtu.be/gok9pvUzZeY" target="_blank"><img src=https://github.com/dusty-nv/jetbot_ros/raw/dev/docs/images/jetbot_gazebo_sim_video.jpg width="750"></a>


