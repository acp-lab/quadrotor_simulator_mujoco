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
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>


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

  void SO3Control();
  static Eigen::Matrix3d quatToRot(const Eigen::Vector4d & q);
  static Eigen::Vector3d vee(const Eigen::Matrix3d & R);

  mj::Simulate *sim_;
  std::string name_prefix, model_param_name;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  std::string world_frame_id_;
  std::string body_frame_id_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_load_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_img_publisher_ptr_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Clock::SharedPtr sim_clock_;

  //void publish_image();
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr actuator_cmd_subscription_;

  std::shared_ptr<Control> actuator_cmds_ptr_;

  std::thread spin_thread;

  // variables system
  Eigen::Matrix<double,3,1> xd_ =
    (Eigen::Matrix<double,3,1>() <<
         0.0, 0.0, 0.0             // desired position  x y z
    ).finished();

  Eigen::Matrix<double,3,1> vd_ =
    (Eigen::Matrix<double,3,1>() <<
         0.0, 0.0, 0.0             // desired velocity  vx vy vz
    ).finished();

  Eigen::Matrix<double,3,1> ad_ =
    (Eigen::Matrix<double,3,1>() <<
         0.0, 0.0, 0.0             // desired acceleration  vx vy vz
    ).finished();

  Eigen::Matrix<double,4,1> qd_ =
    (Eigen::Matrix<double,4,1>() <<
        1.0, 0.0, 0.0, 0.0             // desired quaternion  qw qx qy qz
    ).finished();

  Eigen::Matrix<double,3,1> wd_ =
    (Eigen::Matrix<double,3,1>() <<
         0.0, 0.0, 0.0             // desired angular velocity  wx wy wz
    ).finished();

  double psid_{0.0};

  double g_{9.8};

  Eigen::Matrix<double,3,1> gravityVector_ =
    (Eigen::Matrix<double,3,1>() <<
         0.0, 0.0, g_             // Gravity Vector
    ).finished();

  Eigen::Matrix<double,3,1> ez_ =
    (Eigen::Matrix<double,3,1>() <<
         0.0, 0.0, 1.0             // Unit Vector z
    ).finished();

  Eigen::Matrix<double,3,1> x_ =
    (Eigen::Matrix<double,3,1>() <<
         0.0, 0.0, 0.0             // position  x y z
    ).finished();

  Eigen::Matrix<double,4,1> q_ =
    (Eigen::Matrix<double,4,1>() <<
        1.0, 0.0, 0.0, 0.0             // quaternion  qw qx qy qz
    ).finished();

  Eigen::Matrix<double,3,1> v_ =
    (Eigen::Matrix<double,3,1>() <<
         0.0, 0.0, 0.0             //velocity  vx vy vz
    ).finished();

  Eigen::Matrix<double,3,1> w_ =
    (Eigen::Matrix<double,3,1>() <<
         0.0, 0.0, 0.0             // angular velocity  wx wy wz
    ).finished();

  // Define gains of the controller
  double kp_x_{10.0};
  double kp_y_{10.0};
  double kp_z_{10.0};
  Eigen::Matrix3d KP_ = Eigen::Vector3d(kp_x_, kp_y_, kp_z_).asDiagonal();

  double kv_x_{6.0};
  double kv_y_{6.0};
  double kv_z_{6.0};
  Eigen::Matrix3d KV_ = Eigen::Vector3d(kv_x_, kv_y_, kv_z_).asDiagonal();

  double kw_x_{40.0};
  double kw_y_{40.0};
  double kw_z_{40.0};
  Eigen::Matrix3d KW_ = Eigen::Vector3d(kw_x_, kw_y_, kw_z_).asDiagonal();

  double kq_x_{250.0};
  double kq_y_{250.0};
  double kq_z_{40.0};
  Eigen::Matrix3d KQ_ = Eigen::Vector3d(kq_x_, kq_y_, kq_z_).asDiagonal();

  // Mass of the Quadrotor
  double mass_{1.0 + 0.15};

  Eigen::Matrix3d J_ = Eigen::Vector3d(0.00305587, 0.00159695, 0.00159687).asDiagonal();


};

} // namespace deepbreak
