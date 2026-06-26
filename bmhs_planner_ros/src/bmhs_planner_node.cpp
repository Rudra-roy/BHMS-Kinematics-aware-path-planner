/**
 * @file bmhs_planner_node.cpp
 * @brief ROS2 Action Server for BMHS path planning.
 *
 * Action interface: nav2_msgs/action/ComputePathToPose
 * Publishes:  /bmhs/map (OccupancyGrid), /bmhs/costmap (OccupancyGrid),
 *             /bmhs/path (Path)
 * Subscribes: /odom (Odometry) for current robot pose
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/empty.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "bmhs_planner_ros/map_processor.hpp"
#include "bmhs_planner_ros/vehicle_kinematics.hpp"
#include "bmhs_planner_ros/bmhs_planner.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <cmath>

using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using GoalHandlePlan = rclcpp_action::ServerGoalHandle<ComputePathToPose>;

class BMHSPlannerNode : public rclcpp::Node {
public:
    BMHSPlannerNode()
        : Node("bmhs_planner_node")
    {
        // ─── Declare parameters ──────────────────────────────────────────
        this->declare_parameter<std::string>("map_path", "");
        this->declare_parameter<double>("map_resolution", 0.05);
        this->declare_parameter<double>("map_origin_x", 0.0);
        this->declare_parameter<double>("map_origin_y", 0.0);
        this->declare_parameter<std::string>("vehicle_type", "differential");
        this->declare_parameter<double>("vehicle_width", 0.67);
        this->declare_parameter<double>("vehicle_length", 0.99);
        this->declare_parameter<double>("inflation_radius", 0.3);
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<std::string>("odom_topic", "/a200_0000/odom");

        frame_id_ = this->get_parameter("frame_id").as_string();
        odom_topic_ = this->get_parameter("odom_topic").as_string();

        // ─── Load map ────────────────────────────────────────────────────
        loadMap();

        // ─── Publishers (latched via transient_local QoS) ────────────────
        auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        map_pub_     = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/bmhs/map", latched_qos);
        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/bmhs/costmap", latched_qos);
        path_pub_    = this->create_publisher<nav_msgs::msg::Path>("/bmhs/path", latched_qos);

        publishMapAndCostmap();

        // ─── Odom subscription ───────────────────────────────────────────
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(odom_mutex_);
                latest_odom_ = msg;
                odom_received_ = true;
            }
        );

        // ─── Action server: ComputePathToPose ────────────────────────────
        action_server_ = rclcpp_action::create_server<ComputePathToPose>(
            this,
            "/bmhs/compute_path",
            std::bind(&BMHSPlannerNode::handleGoal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&BMHSPlannerNode::handleCancel, this,
                      std::placeholders::_1),
            std::bind(&BMHSPlannerNode::handleAccepted, this,
                      std::placeholders::_1)
        );

        // ─── Service: reload map ─────────────────────────────────────────
        reload_srv_ = this->create_service<std_srvs::srv::Empty>(
            "/bmhs/reload_map",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                   std::shared_ptr<std_srvs::srv::Empty::Response>) {
                RCLCPP_INFO(this->get_logger(), "Reloading map...");
                loadMap();
                publishMapAndCostmap();
                RCLCPP_INFO(this->get_logger(), "Map reloaded.");
            }
        );

        RCLCPP_INFO(this->get_logger(),
            "BMHS Planner Node ready. Action: /bmhs/compute_path | Odom: %s",
            odom_topic_.c_str());
    }

private:
    // ─── Map loading ─────────────────────────────────────────────────────
    void loadMap() {
        std::string map_path     = this->get_parameter("map_path").as_string();
        double resolution        = this->get_parameter("map_resolution").as_double();
        double vehicle_width     = this->get_parameter("vehicle_width").as_double();
        double vehicle_length    = this->get_parameter("vehicle_length").as_double();
        double inflation_radius  = this->get_parameter("inflation_radius").as_double();

        origin_x_ = this->get_parameter("map_origin_x").as_double();
        origin_y_ = this->get_parameter("map_origin_y").as_double();

        if (map_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'map_path' is empty!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loading map: %s (res=%.3f m/px)", map_path.c_str(), resolution);
        map_proc_ = std::make_shared<bmhs::MapProcessor>();
        map_proc_->load(map_path, resolution, vehicle_width, vehicle_length, inflation_radius);
        RCLCPP_INFO(this->get_logger(), "Map loaded: %dx%d pixels", map_proc_->width(), map_proc_->height());

        // Build kinematics
        std::string vtype = this->get_parameter("vehicle_type").as_string();
        auto kin_type = (vtype == "ackermann")
            ? bmhs::VehicleKinematics::Type::ACKERMANN
            : bmhs::VehicleKinematics::Type::DIFFERENTIAL;

        double tr = 0.0;
        if (kin_type == bmhs::VehicleKinematics::Type::ACKERMANN) {
            tr = vehicle_length / std::tan(30.0 * M_PI / 180.0);
            RCLCPP_INFO(this->get_logger(), "Ackermann turning radius: %.2f m", tr);
        }

        kin_ = std::make_shared<bmhs::VehicleKinematics>(kin_type, tr, resolution);
    }

    // ─── Publish map and costmap ─────────────────────────────────────────
    void publishMapAndCostmap() {
        if (!map_proc_) return;

        auto now = this->get_clock()->now();

        // Raw map
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.stamp = now;
        map_msg.header.frame_id = frame_id_;
        map_msg.info.resolution = map_proc_->resolution();
        map_msg.info.width  = map_proc_->width();
        map_msg.info.height = map_proc_->height();
        map_msg.info.origin.position.x = origin_x_;
        map_msg.info.origin.position.y = origin_y_;
        map_msg.info.origin.position.z = 0.0;
        map_msg.info.origin.orientation.w = 1.0;
        map_msg.data = map_proc_->toOccupancyData();
        map_pub_->publish(map_msg);

        // Costmap (inflated)
        nav_msgs::msg::OccupancyGrid costmap_msg;
        costmap_msg.header = map_msg.header;
        costmap_msg.info   = map_msg.info;
        costmap_msg.data   = map_proc_->toCostmapData();
        costmap_pub_->publish(costmap_msg);

        RCLCPP_INFO(this->get_logger(), "Published map and costmap (%dx%d)",
                     map_proc_->width(), map_proc_->height());
    }

    // ─── Action server callbacks ─────────────────────────────────────────
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const ComputePathToPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(),
            "Received planning request: goal=(%.2f, %.2f)",
            goal->goal.pose.position.x, goal->goal.pose.position.y);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandlePlan>)
    {
        RCLCPP_INFO(this->get_logger(), "Planning cancelled.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandlePlan> goal_handle) {
        // Execute in a separate thread to avoid blocking the action server
        std::thread{std::bind(&BMHSPlannerNode::executePlan, this, goal_handle)}.detach();
    }

    void executePlan(const std::shared_ptr<GoalHandlePlan> goal_handle) {
        auto goal   = goal_handle->get_goal();
        auto result = std::make_shared<ComputePathToPose::Result>();

        if (!map_proc_ || !kin_) {
            RCLCPP_ERROR(this->get_logger(), "Map not loaded!");
            goal_handle->abort(result);
            return;
        }

        // ── Determine start pose ──────────────────────────────────────
        double start_x, start_y, start_theta;

        if (goal->use_start) {
            // Use the start provided in the action goal
            start_x = goal->start.pose.position.x;
            start_y = goal->start.pose.position.y;
            start_theta = tf2::getYaw(goal->start.pose.orientation);
        } else {
            // Use /odom or fallback to map origin
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (odom_received_) {
                start_x = latest_odom_->pose.pose.position.x;
                start_y = latest_odom_->pose.pose.position.y;
                start_theta = tf2::getYaw(latest_odom_->pose.pose.orientation);
                RCLCPP_INFO(this->get_logger(), "Start from odom: (%.2f, %.2f, %.2f)",
                             start_x, start_y, start_theta);
            } else {
                start_x = origin_x_;
                start_y = origin_y_;
                start_theta = 0.0;
                RCLCPP_WARN(this->get_logger(),
                    "No odom received, using map origin as start: (%.2f, %.2f)", start_x, start_y);
            }
        }

        // ── Determine goal pose ───────────────────────────────────────
        double goal_x = goal->goal.pose.position.x;
        double goal_y = goal->goal.pose.position.y;
        double goal_theta = tf2::getYaw(goal->goal.pose.orientation);

        // ── Convert world → pixel coordinates ─────────────────────────
        double res = map_proc_->resolution();
        int h = map_proc_->height();

        // World to pixel (with Y-flip: PGM row 0 is top)
        bmhs::State start_px{
            (start_x - origin_x_) / res,
            static_cast<double>(h - 1) - (start_y - origin_y_) / res,
            start_theta
        };
        bmhs::State goal_px{
            (goal_x - origin_x_) / res,
            static_cast<double>(h - 1) - (goal_y - origin_y_) / res,
            goal_theta
        };

        RCLCPP_INFO(this->get_logger(),
            "Planning: start_px=(%.1f, %.1f) goal_px=(%.1f, %.1f)",
            start_px.x, start_px.y, goal_px.x, goal_px.y);

        // Validate
        if (!map_proc_->isValid(start_px.x, start_px.y)) {
            RCLCPP_ERROR(this->get_logger(), "Start position is in collision or out of bounds!");
            goal_handle->abort(result);
            return;
        }
        if (!map_proc_->isValid(goal_px.x, goal_px.y)) {
            RCLCPP_ERROR(this->get_logger(), "Goal position is in collision or out of bounds!");
            goal_handle->abort(result);
            return;
        }

        // ── Run BMHS planner ──────────────────────────────────────────
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            return;
        }

        bmhs::BMHSPlanner planner(*map_proc_, *kin_);
        auto plan_result = planner.plan(start_px, goal_px);

        if (!plan_result.success) {
            RCLCPP_ERROR(this->get_logger(), "Planning failed after %d iterations.", plan_result.iterations);
            goal_handle->abort(result);
            return;
        }

        // ── Convert pixel path → world coordinates & build Path msg ──
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = frame_id_;

        for (const auto& pt : plan_result.path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;

            // Pixel to world (with Y-flip)
            pose.pose.position.x = pt.x * res + origin_x_;
            pose.pose.position.y = (static_cast<double>(h - 1) - pt.y) * res + origin_y_;
            pose.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, pt.theta);
            pose.pose.orientation = tf2::toMsg(q);

            path_msg.poses.push_back(pose);
        }

        // Publish path for rviz2 visualisation
        path_pub_->publish(path_msg);

        // Populate action result
        result->path = path_msg;
        result->planning_time.sec = static_cast<int32_t>(plan_result.planning_time_s);
        result->planning_time.nanosec = static_cast<uint32_t>(
            (plan_result.planning_time_s - static_cast<int>(plan_result.planning_time_s)) * 1e9);

        RCLCPP_INFO(this->get_logger(),
            "Path planned: %zu poses in %.3fs",
            path_msg.poses.size(), plan_result.planning_time_s);

        goal_handle->succeed(result);
    }

    // ─── Members ─────────────────────────────────────────────────────────
    std::shared_ptr<bmhs::MapProcessor>     map_proc_;
    std::shared_ptr<bmhs::VehicleKinematics> kin_;

    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    std::string frame_id_;
    std::string odom_topic_;

    // Odom
    std::mutex odom_mutex_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    bool odom_received_ = false;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Subscription
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Action server
    rclcpp_action::Server<ComputePathToPose>::SharedPtr action_server_;

    // Service
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reload_srv_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BMHSPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
