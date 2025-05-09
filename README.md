# Quadrotor with Suspended Load Simulator in MuJoCo

This ROS 2 package simulates the behavior of an aerial vehicle using the advanced physics engine [MuJoCo](https://mujoco.org/). The simulator models a quadrotor with a suspeded payload dynamics, RGB camera, IMU and odometry data.

It relies on MuJoCo version 2.3.2, and there is no need to install MuJoCo separately, since the required `.so` files are bundled within the package.

## Dependencies

Install the required system libraries:

```bash
sudo apt update  
sudo apt install libglfw3-dev
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
ros2 launch quadrotor_simulator_mujoco single_quadrotor_payload_sim.launch.py
```

## Available ROS 2 Topics

| Topic Name            | Message Type                | Description                                  |
|------------------------|-----------------------------|----------------------------------------------|
| /quadrotor/cmd         | geometry_msgs/msg/Wrench    | Control input                                |
| /quadrotor/imu         | sensor_msgs/msg/Imu         | Inertial measurement unit data               |
| /quadrotor/odom        | nav_msgs/msg/Odometry       | Full odometry Quadrotor                      |
| /quadrotor/load/odom   | nav_msgs/msg/Odometry       | Full odometry Payload                        |
| /quadrotor/rgb_image   | sensor_msgs/msg/Image       | RGB image stream from onboard camera         |