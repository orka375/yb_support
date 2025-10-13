#include "virtual_hardware_interface/virtual_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <thread>
#include <atomic>

#include <memory>
#include <vector>
#include <string>
#include <functional>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;



// --------------------------------------------------

CallbackReturn VirtualHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("VirtualHardwareInterface"), "Initializing virtual hardware interface...");

  joint_names_.clear();
  for (const auto & joint : info.joints)
  {
    joint_names_.push_back(joint.name);
  }

  joint_positions_.resize(joint_names_.size(), 0.0);
  joint_position_commands_.resize(joint_names_.size(), 0.0);
  joint_velocity_commands_.resize(joint_names_.size(), 0.0);
  joint_velocities_.resize(joint_names_.size(), 0.0);

  node_ = rclcpp::Node::make_shared("virtual_hardware_interface_node");

  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    "Virtual_Hardware_Interface/cmd_vel_out", 10);

  // Create QoS matching the publisher: reliable + transient_local durability
  // QoS: reliable + transient_local durability so we receive latched odom messages
  rclcpp::QoS odom_qos(rclcpp::KeepLast(10));
  odom_qos.reliable();
  odom_qos.transient_local();

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "Mecanum_controller/odom", odom_qos, std::bind(&VirtualHardwareInterface::odom_callback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "Created odom subscription with reliable+transient_local QoS");

  // Create an executor and spin the node in a background thread so callbacks are invoked
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executor_running_.store(true);
  spinner_thread_ = std::thread([this]() {
    while (executor_running_.load()) {
      try {
        executor_->spin_some(std::chrono::milliseconds(100));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node_->get_logger(), "Executor exception: %s", e.what());
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  
  return CallbackReturn::SUCCESS;
}

// --------------------------------------------------

CallbackReturn VirtualHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(node_->get_logger(), "Configured VirtualHardwareInterface");
  return CallbackReturn::SUCCESS;
}

CallbackReturn VirtualHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(node_->get_logger(), "Cleaned up VirtualHardwareInterface");
  // Stop the executor and join the spinner thread if running
  if (executor_running_.load()) {
    executor_running_.store(false);
    if (spinner_thread_.joinable()) {
      spinner_thread_.join();
    }
  }
  if (executor_) {
    try {
      executor_->remove_node(node_);
    } catch (...) {
      // ignore
    }
    executor_.reset();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn VirtualHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(node_->get_logger(), "Activated VirtualHardwareInterface");
  active_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn VirtualHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(node_->get_logger(), "Deactivated VirtualHardwareInterface");
  active_ = false;
  return CallbackReturn::SUCCESS;
}

// --------------------------------------------------

std::vector<hardware_interface::StateInterface> VirtualHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(joint_names_[i], "position", &joint_positions_[i]);
    state_interfaces.emplace_back(joint_names_[i], "velocity", &joint_velocities_[i]);
  }
  return state_interfaces;
}



std::vector<hardware_interface::CommandInterface> VirtualHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(joint_names_[i], "position", &joint_position_commands_[i]);
    command_interfaces.emplace_back(joint_names_[i], "velocity", &joint_velocity_commands_[i]);
  }
  return command_interfaces;
}


// --------------------------------------------------

hardware_interface::return_type VirtualHardwareInterface::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type VirtualHardwareInterface::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // RCLCPP_INFO(node_->get_logger(), " Writing to virtual hardware interface...");
  // RCLCPP_INFO(node_->get_logger(), " Velocity Commands: x=%.2f, y=%.2f, yaw=%.2f", joint_velocity_commands_[0], joint_velocity_commands_[1], joint_velocity_commands_[2]);
  // RCLCPP_INFO(node_->get_logger(), " Position Commands: x=%.2f, y=%.2f, yaw=%.2f", joint_position_commands_[0], joint_position_commands_[1], joint_position_commands_[2]);
  if (!active_) return hardware_interface::return_type::OK;

  geometry_msgs::msg::TwistStamped twist_stamped;
  twist_stamped.header.stamp = node_->now();
  twist_stamped.header.frame_id = "base_link";  // optional, set frame as needed

  if (joint_velocity_commands_.size() >= 3) {
    twist_stamped.twist.linear.x = joint_velocity_commands_[0];
    twist_stamped.twist.linear.y = joint_velocity_commands_[1];
    twist_stamped.twist.angular.z = joint_velocity_commands_[2];
  }

  // If velocity commands are effectively zero, convert position goals into velocity commands
  // using a simple proportional controller.
  auto almost_zero = [](double v){ return std::abs(v) < 1e-6; };
  if (joint_position_commands_.size() >=3 && joint_positions_.size() >=3) {
    if (almost_zero(joint_velocity_commands_[0]) && almost_zero(joint_velocity_commands_[1]) && almost_zero(joint_velocity_commands_[2])) {
      // position errors
      double ex = joint_position_commands_[0] - joint_positions_[0];
      double ey = joint_position_commands_[1] - joint_positions_[1];
      double eyaw = joint_position_commands_[2] - joint_positions_[2];
      // wrap yaw error to [-pi, pi]
      while (eyaw > M_PI) eyaw -= 2.0*M_PI;
      while (eyaw < -M_PI) eyaw += 2.0*M_PI;

      double vx = pos_to_vel_kp_linear_ * ex;
      double vy = pos_to_vel_kp_linear_ * ey;
      double wz = pos_to_vel_kp_angular_ * eyaw;

      // clamp
      auto clamp = [](double v, double lo, double hi){ return std::max(lo, std::min(hi, v)); };
      vx = clamp(vx, -max_linear_vel_, max_linear_vel_);
      vy = clamp(vy, -max_linear_vel_, max_linear_vel_);
      wz = clamp(wz, -max_angular_vel_, max_angular_vel_);

      twist_stamped.twist.linear.x = vx;
      twist_stamped.twist.linear.y = vy;
      twist_stamped.twist.angular.z = wz;
    }
  }
  
  // else {
  //   twist_stamped.twist.linear.x = 0.0;
  //   twist_stamped.twist.linear.y = 0.0;
  //   twist_stamped.twist.angular.z = 0.0;
  // }

  cmd_pub_->publish(twist_stamped);

  return hardware_interface::return_type::OK;
}



// --------------------------------------------------

void VirtualHardwareInterface::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // RCLCPP_INFO(node_->get_logger(), "Callback received odom data");
    if (joint_positions_.size() >= 3 && joint_velocities_.size() >= 3)
    {
        // Linear positions
        joint_positions_[0] = msg->pose.pose.position.x;  // X position
        joint_positions_[1] = msg->pose.pose.position.y;  // Y position
        // joint_positions_[1] = 37.5;  // Y position

        // Yaw angle from quaternion
        double siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                                   msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                        msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        joint_positions_[2] = std::atan2(siny_cosp, cosy_cosp);  // Z yaw

        // Linear velocities
        joint_velocities_[0] = msg->twist.twist.linear.x;
        joint_velocities_[1] = msg->twist.twist.linear.y;

        // Angular velocity around Z
        joint_velocities_[2] = msg->twist.twist.angular.z;
        // RCLCPP_INFO(node_->get_logger(), "Updated joint positions: x=%.2f, y=%.2f, yaw=%.2f", joint_positions_[0], joint_positions_[1], joint_positions_[2]);
    }
}


// --------------------------------------------------

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(VirtualHardwareInterface, hardware_interface::SystemInterface)
