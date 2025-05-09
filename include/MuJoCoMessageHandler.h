#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "rosgraph_msgs/msg/clock.hpp"
#include "geometry_msgs/msg/wrench.hpp"

#include "array_safety.h"
#include "simulate.h"

using namespace rclcpp;

using namespace std::chrono_literals;

namespace deepbreak {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

class MuJoCoMessageHandler : public rclcpp::Node {
public:
  struct ActuatorCmds {
    double time = 0.0;
    std::vector<std::string> actuators_name;
    std::vector<float> kp;
    std::vector<float> pos;
    std::vector<float> kd;
    std::vector<float> vel;
    std::vector<float> torque;
  };

  struct Control {
    double time = 0.0;
    float thrust = 9.81*(1.0 + 0.15);
    float torque_x;
    float torque_y;
    float torque_z;
  };

  MuJoCoMessageHandler(mj::Simulate *sim);
  ~MuJoCoMessageHandler();

  std::shared_ptr<Control> get_actuator_cmds_ptr();

  void publish_image_from_render(const mjvScene* scn,
                                 const mjrContext* con,
                                 const mjrRect& viewport);

private:

  void odom_callback();
  void odom_load_callback();
  void imu_callback();
  void publish_simulation_clock();
  void actuator_cmd_callback(
      const geometry_msgs::msg::Wrench::SharedPtr msg) const;

  void img_callback();

  mj::Simulate *sim_;
  std::string name_prefix, model_param_name;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  std::string world_frame_id_;
  std::string body_frame_id_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_load_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_img_publisher_ptr_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Clock::SharedPtr sim_clock_;

  void publish_image();
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr actuator_cmd_subscription_;

  std::shared_ptr<Control> actuator_cmds_ptr_;

  std::thread spin_thread;
};

} // namespace deepbreak
