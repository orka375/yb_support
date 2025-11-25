#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <my_msgs/srv/execute_motion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>

#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <chrono>
#include <thread>

class MotionNode : public rclcpp::Node
{
public:
    MotionNode(const rclcpp::NodeOptions& options)
        : Node("motion_node", options),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MotionNode...");

        // Subscribe to joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            std::bind(&MotionNode::joint_state_callback, this, std::placeholders::_1));

        // Create ExecuteMotion service
        service_ = this->create_service<my_msgs::srv::ExecuteMotion>(
            "execute_motion",
            std::bind(&MotionNode::execute_motion_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "MotionNode ready and waiting for ExecuteMotion requests.");
    }

    // Initialize MoveGroupInterfaces after construction
    void initialize_move_groups()
    {
        move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "Arm");
        move_group_base_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "Base");
        move_group_robot_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "Robot");
        RCLCPP_INFO(this->get_logger(), "MoveGroupInterfaces initialized.");
    }

private:
    rclcpp::Service<my_msgs::srv::ExecuteMotion>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_base_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_robot_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;

    void execute_motion_callback(
        const std::shared_ptr<my_msgs::srv::ExecuteMotion::Request> request,
        std::shared_ptr<my_msgs::srv::ExecuteMotion::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "[MotionNode] Received motion request: component=%s goal_type=%s",
                    request->component.c_str(), request->goal_type.c_str());

        try
        {
            bool success;
            std::string msg;
            std::tie(success, msg) = run_motion_step(request->component, request->goal_type, request->goal_value, request->goal_pose);

            response->success = success;
            response->message = msg;
        }
        catch (const std::exception &e)
        {
            response->success = false;
            response->message = e.what();
            RCLCPP_ERROR(this->get_logger(), "Motion execution failed: %s", e.what());
        }
    }

    std::pair<bool, std::string> run_motion_step(const std::string &component,
                                                 const std::string &goal_type,
                                                 const std::string &goal_value,
                                                 const geometry_msgs::msg::PoseStamped &goal_pose)
    {
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group;
        if (component == "ARM")
            group = move_group_arm_;
        else if (component == "BASE")
            group = move_group_base_;
        else if (component == "ROBOT")
            group = move_group_robot_;
        else
            return {false, "Unknown component"};

        group->setStartStateToCurrentState();

        if (goal_type == "configuration")
        {
            group->setNamedTarget(goal_value);
        }
        else if (goal_type == "savedpose")
        {
            geometry_msgs::msg::PoseStamped target_pose;
            if (!load_pose_from_file(goal_value, target_pose))
                return {false, "Failed to load pose"};

            auto transformed = transform_pose(target_pose, group->getPlanningFrame());
            if (!transformed)
                return {false, "Pose transformation failed"};

            group->setPoseTarget(*transformed);
        }

        else if (goal_type == "pose")
        {
            group->setPoseTarget(goal_pose);
        }
        else
        {
            return {false, "Unsupported goal_type"};
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success)
            return {false, "Planning failed"};

        auto exec_status = group->execute(plan);
        if (exec_status != moveit::core::MoveItErrorCode::SUCCESS)
            return {false, "Execution failed"};
            
        // Wait for arm joints to stop moving
        if (!wait_for_arm_joints_stopped())
            return {false, "Timeout waiting for arm to stop"};
            
        return {true, "Motion executed successfully"};
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        latest_joint_state_ = msg;
    }

    bool wait_for_arm_joints_stopped(double velocity_threshold = 0.5, double timeout_sec = 10.0)
    {
        auto start_time = this->now();
        rclcpp::Rate rate(10); // Check at 10 Hz

        while (rclcpp::ok())
        {
            // Check for timeout
            if ((this->now() - start_time).seconds() > timeout_sec)
            {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for arm joints to stop");
                return false;
            }

            // Wait for joint state message
            if (!latest_joint_state_)
            {
                rate.sleep();
                continue;
            }

            // Find arm_joint_1 through arm_joint_5 and check their velocities
            bool all_stopped = true;
            std::vector<std::string> arm_joints = {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"};
            
            for (const auto& joint_name : arm_joints)
            {
                auto it = std::find(latest_joint_state_->name.begin(), 
                                   latest_joint_state_->name.end(), 
                                   joint_name);
                
                if (it != latest_joint_state_->name.end())
                {
                    size_t index = std::distance(latest_joint_state_->name.begin(), it);
                    
                    // Check if velocity data is available
                    if (index < latest_joint_state_->velocity.size())
                    {
                        double velocity = latest_joint_state_->velocity[index];
                        if (std::abs(velocity) > velocity_threshold)
                        {
                            all_stopped = false;
                            break;
                        }
                    }
                }
            }

            if (all_stopped)
            {
                RCLCPP_INFO(this->get_logger(), "All arm joints stopped");
                return true;
            }

            rate.sleep();
        }

        return false;
    }

    bool load_pose_from_file(const std::string &pose_name, geometry_msgs::msg::PoseStamped &pose_msg)
    {
        std::string share_dir = ament_index_cpp::get_package_share_directory("youbot");
        std::ifstream file(share_dir + "/data/saved_poses.json");
        if (!file.is_open())
            return false;

        nlohmann::json poses;
        file >> poses;

        if (!poses.contains(pose_name))
            return false;

        auto pose_data = poses[pose_name];
        pose_msg.header.frame_id = pose_data["frame_id"];
        pose_msg.pose.position.x = pose_data["position"]["x"];
        pose_msg.pose.position.y = pose_data["position"]["y"];
        pose_msg.pose.position.z = pose_data["position"]["z"];
        pose_msg.pose.orientation.x = pose_data["orientation"]["x"];
        pose_msg.pose.orientation.y = pose_data["orientation"]["y"];
        pose_msg.pose.orientation.z = pose_data["orientation"]["z"];
        pose_msg.pose.orientation.w = pose_data["orientation"]["w"];

        return true;
    }

    std::optional<geometry_msgs::msg::Pose> transform_pose(const geometry_msgs::msg::PoseStamped &pose_stamped,
                                                           const std::string &target_frame)
    {
        try
        {
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_.lookupTransform(target_frame, pose_stamped.header.frame_id, tf2::TimePointZero);

            geometry_msgs::msg::PoseStamped transformed_pose;
            tf2::doTransform(pose_stamped, transformed_pose, transform);

            return transformed_pose.pose;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "TF transform failed: %s", e.what());
            return std::nullopt;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Node options with use_sim_time
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<MotionNode>(node_options);
    
    // Initialize MoveGroupInterfaces after node is fully constructed
    node->initialize_move_groups();
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
