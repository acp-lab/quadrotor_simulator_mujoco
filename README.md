# Quadrotor Simulator in MuJoCo

This ROS 2 package simulates the behavior of an aerial vehicle using the advanced physics engine [MuJoCo](https://mujoco.org/). The simulator models a quadrotor with suspended load dynamics, RGB camera, and realistic IMU and odometry data.

It relies on MuJoCo version 2.3.2, and there is no need to install MuJoCo separately, since the required `.so` files are bundled within the package.

⚠️ Important: This package should be placed inside the `/src` folder of your ROS 2 workspace.

## Dependencies

Install the required system libraries:

```bash
sudo apt update  
sudo apt install libglfw3-dev
```

Install ROS 2 Python and visualization dependencies (if not already installed):

```bash
sudo apt install python3-colcon-common-extensions  
sudo apt install ros-humble-rqt-image-view  
sudo apt install ros-humble-cv-bridge  
sudo apt install ros-humble-image-transport
```
## Installation

Clone the package into your ROS 2 workspace src folder:

```bash
cd ~/ros2_ws/src  
git clone https://github.com/acp-lab/quadrotor_simulator_mujoco.git
```
Then build the workspace:

```bash
cd ~/ws  
colcon build --symlink-install  
source install/setup.bash
```

## Execution

Launch the simulator using:

```bash
ros2 launch quadrotor_simulator_mujoco single_quadrotor_sim.launch.py
```

## Available ROS 2 Topics

/quadrotor/cmd          - geometry_msgs/msg/Wrench      - Command input (force and torque)  
/quadrotor/imu          - sensor_msgs/msg/Imu           - Inertial measurement unit data  
/quadrotor/load         - nav_msgs/msg/Odometry         - Pose and twist of the suspended load  
/quadrotor/odom         - nav_msgs/msg/Odometry         - Full odometry (position, velocity, pose)  
/quadrotor/rgb_image    - sensor_msgs/msg/Image         - RGB image stream from onboard camera