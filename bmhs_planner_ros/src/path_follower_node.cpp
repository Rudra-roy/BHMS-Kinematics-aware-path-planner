/**
 * @file path_follower_node.cpp
 * @brief ROS2 Action Server implementing pure-pursuit path following.
 *
 * Action interface: nav2_msgs/action/FollowPath
 * Subscribes: odom topic (configurable, default /a200_0000/odom)
 * Publishes:  cmd_vel topic (configurable, default /a200_0000/cmd_vel)
 *
 * Provides feedback: distance_to_goal, speed.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/follow_path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <mutex>
#include <memory>

using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandleFollow = rclcpp_action::ServerGoalHandle<FollowPath>;

class PathFollowerNode : public rclcpp::Node {
public:
    PathFollowerNode()
        : Node("path_follower_node")
    {
        // ─── Parameters ──────────────────────────────────────────────────
        this->declare_parameter<double>("lookahead_distance", 0.8);
        this->declare_parameter<double>("max_linear_vel", 0.5);
        this->declare_parameter<double>("max_angular_vel", 1.0);
        this->declare_parameter<double>("goal_tolerance", 0.3);
        this->declare_parameter<std::string>("odom_topic", "/a200_0000/odom");
        this->declare_parameter<std::string>("cmd_vel_topic", "/a200_0000/cmd_vel");
        this->declare_parameter<double>("control_rate", 20.0);

        lookahead_  = this->get_parameter("lookahead_distance").as_double();
        max_lin_    = this->get_parameter("max_linear_vel").as_double();
        max_ang_    = this->get_parameter("max_angular_vel").as_double();
        goal_tol_   = this->get_parameter("goal_tolerance").as_double();
        ctrl_rate_  = this->get_parameter("control_rate").as_double();

        std::string odom_topic   = this->get_parameter("odom_topic").as_string();
        std::string cmdvel_topic = this->get_parameter("cmd_vel_topic").as_string();

        // ─── Odom subscription ───────────────────────────────────────────
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(odom_mutex_);
                latest_odom_ = msg;
                odom_received_ = true;
            }
        );

        // ─── Cmd_vel publisher ───────────────────────────────────────────
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmdvel_topic, 10);

        // ─── Action server: FollowPath ───────────────────────────────────
        action_server_ = rclcpp_action::create_server<FollowPath>(
            this,
            "/bmhs/follow_path",
            std::bind(&PathFollowerNode::handleGoal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&PathFollowerNode::handleCancel, this,
                      std::placeholders::_1),
            std::bind(&PathFollowerNode::handleAccepted, this,
                      std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(),
            "Path Follower ready. Action: /bmhs/follow_path | cmd_vel: %s | odom: %s",
            cmdvel_topic.c_str(), odom_topic.c_str());
    }

private:
    // ─── Action callbacks ────────────────────────────────────────────────
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const FollowPath::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(),
            "Received path to follow: %zu poses", goal->path.poses.size());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleFollow>)
    {
        RCLCPP_INFO(this->get_logger(), "Path following cancelled.");
        stopRobot();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleFollow> goal_handle) {
        std::thread{std::bind(&PathFollowerNode::executeFollow, this, goal_handle)}.detach();
    }

    // ─── Pure-pursuit controller ─────────────────────────────────────────
    void executeFollow(const std::shared_ptr<GoalHandleFollow> goal_handle) {
        auto goal = goal_handle->get_goal();
        const auto& poses = goal->path.poses;

        if (poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty path received.");
            auto result = std::make_shared<FollowPath::Result>();
            goal_handle->succeed(result);
            return;
        }

        rclcpp::Rate rate(ctrl_rate_);
        size_t closest_idx = 0;

        while (rclcpp::ok()) {
            // Check cancellation
            if (goal_handle->is_canceling()) {
                stopRobot();
                auto result = std::make_shared<FollowPath::Result>();
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Follow path cancelled.");
                return;
            }

            // Get current pose from odom
            double robot_x, robot_y, robot_yaw;
            {
                std::lock_guard<std::mutex> lock(odom_mutex_);
                if (!odom_received_) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Waiting for odom...");
                    rate.sleep();
                    continue;
                }
                robot_x   = latest_odom_->pose.pose.position.x;
                robot_y   = latest_odom_->pose.pose.position.y;
                robot_yaw = tf2::getYaw(latest_odom_->pose.pose.orientation);
            }

            // Check if we've reached the final goal
            const auto& final_pose = poses.back().pose.position;
            double dist_to_goal = std::hypot(robot_x - final_pose.x, robot_y - final_pose.y);

            if (dist_to_goal < goal_tol_) {
                stopRobot();
                RCLCPP_INFO(this->get_logger(), "Goal reached! (dist=%.3f)", dist_to_goal);
                auto result = std::make_shared<FollowPath::Result>();
                goal_handle->succeed(result);
                return;
            }

            // Find closest point on path
            double min_dist = std::numeric_limits<double>::max();
            for (size_t i = closest_idx; i < poses.size(); ++i) {
                double d = std::hypot(
                    robot_x - poses[i].pose.position.x,
                    robot_y - poses[i].pose.position.y);
                if (d < min_dist) {
                    min_dist = d;
                    closest_idx = i;
                }
            }

            // Find lookahead point
            size_t lookahead_idx = closest_idx;
            for (size_t i = closest_idx; i < poses.size(); ++i) {
                double d = std::hypot(
                    robot_x - poses[i].pose.position.x,
                    robot_y - poses[i].pose.position.y);
                if (d >= lookahead_) {
                    lookahead_idx = i;
                    break;
                }
                lookahead_idx = i;  // default to farthest if none beyond lookahead
            }

            // Lookahead target
            double lx = poses[lookahead_idx].pose.position.x;
            double ly = poses[lookahead_idx].pose.position.y;

            // Pure-pursuit: compute curvature
            // Transform lookahead point to robot frame
            double dx = lx - robot_x;
            double dy = ly - robot_y;
            double local_x =  dx * std::cos(robot_yaw) + dy * std::sin(robot_yaw);
            double local_y = -dx * std::sin(robot_yaw) + dy * std::cos(robot_yaw);

            double ld = std::hypot(local_x, local_y);
            if (ld < 0.01) ld = 0.01;  // avoid division by zero

            double curvature = 2.0 * local_y / (ld * ld);

            // Compute velocities
            double linear_vel = max_lin_;

            // Slow down when close to goal
            if (dist_to_goal < lookahead_ * 2.0) {
                linear_vel = std::max(0.1, max_lin_ * dist_to_goal / (lookahead_ * 2.0));
            }

            // Slow down on sharp turns
            double abs_curv = std::abs(curvature);
            if (abs_curv > 0.5) {
                linear_vel *= std::max(0.3, 1.0 - abs_curv * 0.5);
            }

            double angular_vel = linear_vel * curvature;

            // Clamp
            linear_vel  = std::clamp(linear_vel,  -max_lin_, max_lin_);
            angular_vel = std::clamp(angular_vel, -max_ang_, max_ang_);

            // Publish cmd_vel
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x  = linear_vel;
            cmd.angular.z = angular_vel;
            cmd_pub_->publish(cmd);

            // Publish feedback
            auto feedback = std::make_shared<FollowPath::Feedback>();
            feedback->distance_to_goal = static_cast<float>(dist_to_goal);
            feedback->speed = static_cast<float>(linear_vel);
            goal_handle->publish_feedback(feedback);

            rate.sleep();
        }
    }

    void stopRobot() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }

    // ─── Members ─────────────────────────────────────────────────────────
    double lookahead_, max_lin_, max_ang_, goal_tol_, ctrl_rate_;

    std::mutex odom_mutex_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    bool odom_received_ = false;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp_action::Server<FollowPath>::SharedPtr action_server_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
