#include <mujoco/mujoco.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class MuJoCoROS2Plugin : public rclcpp::Node {
public:
    MuJoCoROS2Plugin() : Node("mujoco_ros2_plugin") {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("joint_position", 10);
    }

    void publish_joint_position(double position) {
        auto message = std_msgs::msg::Float64();
        message.data = position;
        publisher_->publish(message);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};

std::shared_ptr<MuJoCoROS2Plugin> plugin_node;

extern "C" void mj_plugin_init() {
    rclcpp::init(0, nullptr);
    plugin_node = std::make_shared<MuJoCoROS2Plugin>();
    RCLCPP_INFO(plugin_node->get_logger(), "MuJoCo ROS2 Plugin initialized");
}

extern "C" void mj_plugin_step(mjModel* m, mjData* d) {
    // Assuming the joint is at index 0
    double joint_position = d->qpos[0];
    plugin_node->publish_joint_position(joint_position);
}

extern "C" void mj_plugin_close() {
    RCLCPP_INFO(plugin_node->get_logger(), "MuJoCo ROS2 Plugin shutting down");
    rclcpp::shutdown();
}