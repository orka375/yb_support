#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <string>
#include <thread>
#include <atomic>

class VirtualHardwareInterface : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;


private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);


  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spinner_thread_;
  std::atomic<bool> executor_running_{false};
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;   // <--- new
  std::vector<double> joint_position_commands_;
  std::vector<double> joint_velocity_commands_;  // <--- new
  std::vector<std::string> joint_names_;
  bool active_{false};
  // Parameters for converting position goals into velocity commands
  double pos_to_vel_kp_linear_{1.0};
  double pos_to_vel_kp_angular_{1.0};
  double max_linear_vel_{0.2};
  double max_angular_vel_{0.4};

};