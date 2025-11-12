#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <my_msgs/srv/execute_motion.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <nlohmann/json.hpp>
#include <optional>
#include <memory>

using json = nlohmann::json;

enum class COMPONENTS { ARM, BASE, ROBOT };
enum class GOAL_TYPES { MOVEITCONFIG, POSE, STATE, SAVEDPOSE };

class MoveItMotionNode
    : public rclcpp::Node,
      public std::enable_shared_from_this<MoveItMotionNode>
{
public:
    MoveItMotionNode(const rclcpp::NodeOptions& options)
        : Node("motion_helper_node", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MoveItMotionNode...");

        execute_motion_srv_ = this->create_service<my_msgs::srv::ExecuteMotion>(
            "execute_motion",
            std::bind(&MoveItMotionNode::execute_motion_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

  

        RCLCPP_INFO(this->get_logger(), "MoveItMotionNode initialized successfully.");
    }

    void set_move_group_interface(
        std::string component,
        moveit::planning_interface::MoveGroupInterfacePtr move_group)
    {   
        if (component == "Arm") {
            component_ = COMPONENTS::ARM;
            move_group_arm_ = move_group;
        } else if (component == "Base") {
            component_ = COMPONENTS::BASE;
            move_group_base_ = move_group;
        } else if (component == "Robot") {
            component_ = COMPONENTS::ROBOT;
            move_group_robot_ = move_group;
        }
    }

    

private:
    COMPONENTS component_;
    // moveit::planning_interface::MoveGroupInterfacePtr move_group_arm_;
    // moveit::planning_interface::MoveGroupInterfacePtr move_group_base_;
    // moveit::planning_interface::MoveGroupInterfacePtr move_group_robot_;



    //===============================================================================
    void execute_motion_callback(
        const std::shared_ptr<my_msgs::srv::ExecuteMotion::Request> request,
        std::shared_ptr<my_msgs::srv::ExecuteMotion::Response> response)
    {
        try
        {
            COMPONENTS comp = string_to_component(request->component);
            GOAL_TYPES goal_type = string_to_goal_type(request->goal_type);

            bool success;
            std::string message;

            std::tie(success, message) = run_motion_step(comp, request->step_name, goal_type, request->goal_value);

            response->success = success;
            response->message = message;
        }
        catch (const std::exception &e)
        {
            response->success = false;
            response->message = e.what();
        }
    }

    
    std::pair<bool, std::string> run_motion_step(COMPONENTS component,
                                                const std::string &step_name,
                                                GOAL_TYPES goal_type,
                                                const std::string &goal_value)
    {
        RCLCPP_INFO(this->get_logger(), "Running step '%s'", step_name.c_str());

        moveit::planning_interface::MoveGroupInterfacePtr planning_component;
        std::string planning_frame = "world";

        switch (component)
        {
        case COMPONENTS::ARM:
            planning_component = move_group_arm_;
            planning_frame = "pen";
            break;
        case COMPONENTS::BASE:
            planning_component = move_group_base_;
            planning_frame = "base_link";
            break;
        case COMPONENTS::ROBOT:
            planning_component = move_group_robot_;
            planning_frame = "pen";
            break;
        default:
            return {false, "Unsupported component"};
        }
        planning_component->getCurrentState();
        planning_component->setStartStateToCurrentState();

        // Set goal
        if (goal_type == GOAL_TYPES::MOVEITCONFIG)
        {
            planning_component->setNamedTarget(goal_value);
        }
        else if (goal_type == GOAL_TYPES::SAVEDPOSE)
        {
            geometry_msgs::msg::PoseStamped pose_msg;
            if (!load_saved_pose(goal_value, pose_msg))
                return {false, "Saved pose not found"};

            auto transformed_pose = transform_pose(pose_msg, planning_frame);
            if (!transformed_pose)
                return {false, "Pose transformation failed"};

            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = planning_frame;
            target_pose.pose = *transformed_pose;

            planning_component->setPoseTarget(target_pose);
        }
        else
        {
            return {false, "Goal type not implemented"};
        }

        // Plan
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (planning_component->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success)
            return {false, "Planning failed"};

        // Execute
        moveit::core::MoveItErrorCode exec_status = planning_component->execute(plan);
        if (exec_status != moveit::core::MoveItErrorCode::SUCCESS)
            return {false, "Execution failed"};

        return {true, "Step executed successfully"};
    }

    //===============================================================================


  
    bool load_saved_pose(const std::string &pose_name, geometry_msgs::msg::PoseStamped &pose_msg)
    {
        std::string share_dir = ament_index_cpp::get_package_share_directory("youbot");
        std::ifstream file(share_dir + "/data/saved_poses.json");
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open saved_poses.json");
            return false;
        }

        json poses;
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

    // Transform pose using TF2
    std::optional<geometry_msgs::msg::Pose> transform_pose(
        const geometry_msgs::msg::PoseStamped &pose_stamped,
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

    // Convert string to COMPONENTS enum
    COMPONENTS string_to_component(const std::string &s)
    {
        if (s == "ARM") return COMPONENTS::ARM;
        if (s == "BASE") return COMPONENTS::BASE;
        if (s == "ROBOT") return COMPONENTS::ROBOT;
        throw std::runtime_error("Unknown component: " + s);
    }

    // Convert string to GOAL_TYPES enum
    GOAL_TYPES string_to_goal_type(const std::string &s)
    {
        if (s == "configuration") return GOAL_TYPES::MOVEITCONFIG;
        if (s == "pose") return GOAL_TYPES::POSE;
        if (s == "state") return GOAL_TYPES::STATE;
        if (s == "savedpose") return GOAL_TYPES::SAVEDPOSE;
        throw std::runtime_error("Unknown goal type: " + s);
    }


    rclcpp::Service<my_msgs::srv::ExecuteMotion>::SharedPtr execute_motion_srv_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    moveit::planning_interface::MoveGroupInterfacePtr move_group_arm_;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_base_;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_robot_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create node with options to automatically declare parameters from overrides (including use_sim_time)
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    
    auto node = std::make_shared<MoveItMotionNode>(node_options);

    // Create MoveGroupInterface and assign it to the node
    auto component_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "Arm");
    auto component_base = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "Base");
    auto component_robot = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "Robot");
    node->set_move_group_interface("Arm",component_arm);
    node->set_move_group_interface("Base",component_base);
    node->set_move_group_interface("Robot",component_robot);

    // Spin
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

