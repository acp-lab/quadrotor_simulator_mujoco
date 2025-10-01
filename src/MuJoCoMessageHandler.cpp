#include "MuJoCoMessageHandler.h"
#include "sensor_msgs/image_encodings.hpp"
#include "geometry_msgs/msg/wrench.hpp"

namespace deepbreak {

MuJoCoMessageHandler::MuJoCoMessageHandler(mj::Simulate *sim)
    : Node("MuJoCoMessageHandler"), sim_(sim), name_prefix("simulation/") {
  model_param_name = name_prefix + "model_file";
  // Launch parameters
  this->declare_parameter(model_param_name, "");
  this->declare_parameter<double>("rate_odom", 200.0);
  this->declare_parameter<double>("rate_imu", 200.0);

  // Mapping to sample time
  // Retrieve the value
  double rate_odom;
  this->get_parameter("rate_odom", rate_odom);

  double rate_imu;
  this->get_parameter("rate_imu", rate_imu);

  this->declare_parameter<std::string>("world_frame_id", "world");
  this->get_parameter("world_frame_id", world_frame_id_);

  this->declare_parameter<std::string>("body_frame_id", "quadrotor");
  this->get_parameter("body_frame_id", body_frame_id_);
  
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
  
  // Create publishers
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
  odom_publisher_load_ = this->create_publisher<nav_msgs::msg::Odometry>("load/odom", rclcpp::QoS(10));
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(10));
  
  clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 100);

  // Timer since we are using Mujoco
  timers_.emplace_back(this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_odom), std::bind(&MuJoCoMessageHandler::odom_callback, this)));

  timers_.emplace_back(this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_odom), std::bind(&MuJoCoMessageHandler::odom_load_callback, this)));

  timers_.emplace_back(this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_imu), std::bind(&MuJoCoMessageHandler::imu_callback, this)));

  timers_.emplace_back(this->create_wall_timer(
    1ms, std::bind(&MuJoCoMessageHandler::publish_simulation_clock, this)));
  
  // Create subscriber to the cmd commands
  actuator_cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Wrench>("cmd", qos, std::bind(&MuJoCoMessageHandler::actuator_cmd_callback, this, std::placeholders::_1));

  // Message to show that we are using Mujoco
  actuator_cmds_ptr_ = std::make_shared<Control>();
  RCLCPP_INFO(this->get_logger(), "Start MuJoCoMessageHandler ...");

  sim_->uiloadrequest.fetch_add(1);
}

MuJoCoMessageHandler::~MuJoCoMessageHandler() {
  RCLCPP_INFO(this->get_logger(), "close node ...");
}

void MuJoCoMessageHandler::odom_callback() {
  const std::lock_guard<std::mutex> lock(sim_->mtx);
  if (sim_->d != nullptr) {
    auto message = nav_msgs::msg::Odometry();
    message.header.stamp = this->now();
    message.header.frame_id =  world_frame_id_;
    message.child_frame_id = body_frame_id_;


    message.pose.pose.position.x = sim_->d->qpos[0];
    message.pose.pose.position.y = sim_->d->qpos[1];
    message.pose.pose.position.z = sim_->d->qpos[2];

    message.pose.pose.orientation.w = sim_->d->qpos[3];
    message.pose.pose.orientation.x = sim_->d->qpos[4];
    message.pose.pose.orientation.y = sim_->d->qpos[5];
    message.pose.pose.orientation.z = sim_->d->qpos[6];

    // Velocities body frame
    for (int i = 0; i < sim_->m->nsensor; i++) {
      if (sim_->m->sensor_type[i] == mjtSensor::mjSENS_VELOCIMETER) {
          message.twist.twist.linear.x = sim_->d->sensordata[sim_->m->sensor_adr[i]];
          message.twist.twist.linear.y = sim_->d->sensordata[sim_->m->sensor_adr[i] +1];
          message.twist.twist.linear.z = sim_->d->sensordata[sim_->m->sensor_adr[i] + 2];
      } 
      if (sim_->m->sensor_type[i] == mjtSensor::mjSENS_GYRO) {
          message.twist.twist.angular.x = sim_->d->sensordata[sim_->m->sensor_adr[i]];
          message.twist.twist.angular.y = sim_->d->sensordata[sim_->m->sensor_adr[i] + 1];
          message.twist.twist.angular.z = sim_->d->sensordata[sim_->m->sensor_adr[i] + 2];
      } 
    }
    odom_publisher_->publish(message);
  }
}

void MuJoCoMessageHandler::odom_load_callback() {

  const std::lock_guard<std::mutex> lock(sim_->mtx);
  if (sim_->d != nullptr) {
    auto message = nav_msgs::msg::Odometry();
    message.header.stamp = this->now();
    message.header.frame_id = world_frame_id_;

    message.pose.pose.position.x = sim_->d->qpos[7];
    message.pose.pose.position.y = sim_->d->qpos[8];
    message.pose.pose.position.z = sim_->d->qpos[9];

    message.pose.pose.orientation.w = sim_->d->qpos[10];
    message.pose.pose.orientation.x = sim_->d->qpos[11];
    message.pose.pose.orientation.y = sim_->d->qpos[12];
    message.pose.pose.orientation.z = sim_->d->qpos[13];

    message.twist.twist.linear.x = sim_->d->qvel[6];
    message.twist.twist.linear.y = sim_->d->qvel[7];
    message.twist.twist.linear.z = sim_->d->qvel[8];

    message.twist.twist.angular.x = sim_->d->qvel[9];
    message.twist.twist.angular.y = sim_->d->qvel[10];
    message.twist.twist.angular.z = sim_->d->qvel[11];
    odom_publisher_load_->publish(message);
  }
}

void MuJoCoMessageHandler::actuator_cmd_callback(
    const geometry_msgs::msg::Wrench::SharedPtr msg) const {
  if (sim_->d != nullptr) {
    //actuator_cmds_ptr_->time = this->now();

    actuator_cmds_ptr_->thrust = msg->force.z;
    actuator_cmds_ptr_->torque_x = msg->torque.x;
    actuator_cmds_ptr_->torque_y = msg->torque.y;
    actuator_cmds_ptr_->torque_z = msg->torque.z;

  }
}

void MuJoCoMessageHandler::imu_callback() {
  if (sim_->d != nullptr) {
    auto message = sensor_msgs::msg::Imu();
    message.header.frame_id = body_frame_id_;
    message.header.stamp = this->now();
    const std::lock_guard<std::mutex> lock(sim_->mtx);

    for (int i = 0; i < sim_->m->nsensor; i++) {
      if (sim_->m->sensor_type[i] == mjtSensor::mjSENS_ACCELEROMETER) {
        message.linear_acceleration.x =
            sim_->d->sensordata[sim_->m->sensor_adr[i]];
        message.linear_acceleration.y =
            sim_->d->sensordata[sim_->m->sensor_adr[i] + 1];
        message.linear_acceleration.z =
            sim_->d->sensordata[sim_->m->sensor_adr[i] + 2] - 9.81;
      } else if (sim_->m->sensor_type[i] == mjtSensor::mjSENS_FRAMEQUAT) {
        message.orientation.w = sim_->d->sensordata[sim_->m->sensor_adr[i]];
        message.orientation.x = sim_->d->sensordata[sim_->m->sensor_adr[i] + 1];
        message.orientation.y = sim_->d->sensordata[sim_->m->sensor_adr[i] + 2];
        message.orientation.z = sim_->d->sensordata[sim_->m->sensor_adr[i] + 3];
      } else if (sim_->m->sensor_type[i] == mjtSensor::mjSENS_GYRO) {
        message.angular_velocity.x =
            sim_->d->sensordata[sim_->m->sensor_adr[i]];
        message.angular_velocity.y =
            sim_->d->sensordata[sim_->m->sensor_adr[i] + 1];
        message.angular_velocity.z =
            sim_->d->sensordata[sim_->m->sensor_adr[i] + 2];
      }
    }
    imu_publisher_->publish(message);
  }
}

void MuJoCoMessageHandler::publish_simulation_clock() {
    if (sim_ && sim_->d) {
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock = rclcpp::Time(sim_->d->time * 1e9);  // seconds → nanoseconds
        clock_pub_->publish(clock_msg);
    }
}

std::shared_ptr<MuJoCoMessageHandler::Control>
MuJoCoMessageHandler::get_actuator_cmds_ptr() {
  return actuator_cmds_ptr_;
}
} // namespace deepbreak
