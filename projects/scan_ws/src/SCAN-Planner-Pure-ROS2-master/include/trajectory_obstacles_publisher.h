/**
 * @file      trajectory_obstacles_publisher.h
 * @brief     SCAN-Planner ROS 2 trajectory and obstacle visualization node.
 * @author    juchunyu <juchunyu@qq.com>
 * @date      2026-07-23 20:00:01
 * @copyright Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC).
 *            All rights reserved.
 * This header declares interactive path input, planning, and visualization interfaces.
 */

#ifndef TRAJECTORY_OBSTACLES_PUBLISHER_H_
#define TRAJECTORY_OBSTACLES_PUBLISHER_H_

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planner_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker_array.hpp"

class TrajectoryAndObstaclesPublisher : public rclcpp::Node
{
public:
    TrajectoryAndObstaclesPublisher();
    ~TrajectoryAndObstaclesPublisher() = default;

private:
    void init_scan_planner_base();
    void publish_and_plan();

    // 回调函数
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void rviz_global_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void rviz_obstacles_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void rviz_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void pose_estimate_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void trigger_plan_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // 发布函数
    void publish_global_path();
    void publish_planned_trajectory();
    void publish_traveled_path();
    void publish_obstacles();
    void publish_elevation_cloud();
    void publish_a_star_path();
    void publish_local_obstacles();

    // 辅助函数
    void generate_straight_path(const geometry_msgs::msg::PoseStamped& start,
                                const geometry_msgs::msg::PoseStamped& goal);
    void add_obstacle_at_position(double x, double y);

    void discretize_trajectory(
        const std::vector<scan_planner::PathPoint>& original_trajectory,
        std::vector<scan_planner::PathPoint>& discrete_trajectory,
        double interval = 0.1);

    double distance(const scan_planner::PathPoint& p1,
                    const scan_planner::PathPoint& p2);

    void makeInflatedVoxelMarker(const std::vector<Eigen::Vector3d>& points,
                                 double voxel_size);
    void makePointCloud2(const std::vector<Eigen::Vector3d>& points);
    void makeInflatedVoxelEdgeMarker(const std::vector<Eigen::Vector3d>& points,
                                     double voxel_size);

    // ROS 2 发布者/订阅者
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr a_star_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_traj_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traveled_path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obs_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr elevation_cloud_pub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inflated_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr inflated_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr inflated_edge_marker_pub_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr rviz_global_path_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rviz_obstacles_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rviz_point_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_estimate_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_plan_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // SCAN-Planner核心对象
    std::shared_ptr<scan_planner::PlannerInterface> scan_planner_;

    // 数据存储
    std::vector<scan_planner::PathPoint> global_plan_traj_;
    std::vector<scan_planner::ObstacleInfo> obstacles_;
    std::mutex data_mutex_;
    bool has_valid_global_path_;
    bool has_obstacles_;
    bool should_plan_;
    bool needs_replan_;  // 新增：是否需要重新规划的标志
    bool has_cloud_;
    bool has_odom_;
    bool has_global_path_;
    bool flag_ = false;
    std::vector<scan_planner::PathPoint> planned_traj;
    std::vector<scan_planner::PathPoint> traveled_traj_;

    scan_planner::PathPoint cur_pose_{0.0F, 0.0F, 0.2F, 0.0F, 0.0F};

    std::vector<std::vector<Eigen::Vector3d>> a_star_pathes_;

    // 参数
    double max_vel_ = 2.0;
    double max_acc_ = 3.0;
    double max_jerk_ = 4.0;
    double map_resolution_ = 0.1;
    double map_x_size_ = 50.0;
    double map_y_size_ = 50.0;
    double map_z_size_ = 5.0;
    Eigen::Vector3d map_origin_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    double map_inflate_value_ = 1.0;

    std::string cloud_topic_ = "/robot_0/cloud_registered_world";
    std::string odom_topic_ = "/robot_0/odometry";
    std::string global_path_topic_ = "/scan_global_path";
    std::string planning_frame_ = "world";
    double local_range_x_ = 4.0;
    double local_range_y_ = 4.0;
    double local_range_z_down_ = 1.0;
    double local_range_z_up_ = 1.5;
    double self_filter_x_ = 0.35;
    double self_filter_y_ = 0.35;
    double self_filter_z_ = 1.10;
    double voxel_size_ = 0.08;
    int max_obstacle_points_ = 60000;
    double traveled_path_min_step_ = 0.05;
    double elevation_grid_resolution_ = 0.12;
    double planning_period_ms_ = 100.0;

    double theta_ = 0.0;
};

#endif  // TRAJECTORY_OBSTACLES_PUBLISHER_H_
