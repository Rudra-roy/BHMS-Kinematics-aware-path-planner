/**
 * @file bmhs_navigator_node.cpp
 * @brief ROS2 Coordinator node — bridges rviz2 "2D Goal Pose" to the BMHS
 *        planning and following pipeline.
 *
 * Subscribes: /goal_pose (from rviz2 "2D Goal Pose" tool)
 * Action clients: /bmhs/compute_path (ComputePathToPose)
 *                 /bmhs/follow_path  (FollowPath)
 *
 * Workflow: goal_pose → plan path → follow path.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <string>

using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using FollowPath = nav2_msgs::action::FollowPath;

class BMHSNavigatorNode : public rclcpp::Node {
public:
    BMHSNavigatorNode()
        : Node("bmhs_navigator_node")
    {
        this->declare_parameter<std::string>("goal_topic", "/goal_pose");
        std::string goal_topic = this->get_parameter("goal_topic").as_string();

        // ─── Goal subscription (from rviz2) ──────────────────────────────
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic, 10,
            std::bind(&BMHSNavigatorNode::goalCallback, this, std::placeholders::_1)
        );

        // ─── Action clients ──────────────────────────────────────────────
        plan_client_ = rclcpp_action::create_client<ComputePathToPose>(
            this, "/bmhs/compute_path");
        follow_client_ = rclcpp_action::create_client<FollowPath>(
            this, "/bmhs/follow_path");

        RCLCPP_INFO(this->get_logger(),
            "BMHS Navigator ready. Listening on: %s", goal_topic.c_str());
    }

private:
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(),
            "Goal received from rviz2: (%.2f, %.2f)",
            msg->pose.position.x, msg->pose.position.y);

        // Cancel any active follow action
        if (follow_active_) {
            RCLCPP_INFO(this->get_logger(), "Cancelling previous path follow...");
            follow_client_->async_cancel_all_goals();
            follow_active_ = false;
        }

        // Wait for planner action server
        if (!plan_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Planner action server not available!");
            return;
        }

        // Send planning goal
        auto plan_goal = ComputePathToPose::Goal();
        plan_goal.goal = *msg;
        plan_goal.use_start = false;  // Use robot's current odom

        RCLCPP_INFO(this->get_logger(), "Sending planning request...");

        auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&BMHSNavigatorNode::planResultCallback, this, std::placeholders::_1);

        plan_client_->async_send_goal(plan_goal, send_goal_options);
    }

    void planResultCallback(
        const rclcpp_action::ClientGoalHandle<ComputePathToPose>::WrappedResult& result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(),
                    "Path planned successfully: %zu poses (%.3fs)",
                    result.result->path.poses.size(),
                    result.result->planning_time.sec +
                    result.result->planning_time.nanosec * 1e-9);
                // Now send the path to the follower
                sendFollowPath(result.result->path);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Planning failed (aborted).");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Planning was cancelled.");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown planning result.");
                break;
        }
    }

    void sendFollowPath(const nav_msgs::msg::Path& path) {
        if (!follow_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Follower action server not available!");
            return;
        }

        auto follow_goal = FollowPath::Goal();
        follow_goal.path = path;

        RCLCPP_INFO(this->get_logger(), "Sending path to follower...");

        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
        send_goal_options.feedback_callback =
            [this](auto, const std::shared_ptr<const FollowPath::Feedback> fb) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                    "Following: dist_to_goal=%.2f speed=%.2f",
                    fb->distance_to_goal, fb->speed);
            };
        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<FollowPath>::WrappedResult& res) {
                follow_active_ = false;
                switch (res.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Goal reached!");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Path following failed.");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "Path following cancelled.");
                        break;
                    default:
                        break;
                }
            };

        follow_client_->async_send_goal(follow_goal, send_goal_options);
        follow_active_ = true;
    }

    // ─── Members ─────────────────────────────────────────────────────────
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp_action::Client<ComputePathToPose>::SharedPtr plan_client_;
    rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
    bool follow_active_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BMHSNavigatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
