/**
 * @file      trajectory_publisher.cpp
 * @brief     SCAN-Planner ROS 2 trajectory and obstacle visualization implementation.
 * @author    juchunyu <juchunyu@qq.com>
 * @date      2026-07-23 20:00:01
 * @copyright Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC).
 *            All rights reserved.
 * This source implements interactive path input, planning, and visualization behavior.
 */

#include "trajectory_obstacles_publisher.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/utils.h"

using scan_planner::ObstacleInfo;
using scan_planner::PathPoint;
using scan_planner::PlannerInterface;

namespace
{
struct VoxelKey
{
    std::int64_t x;
    std::int64_t y;
    std::int64_t z;

    bool operator==(const VoxelKey& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash
{
    std::size_t operator()(const VoxelKey& key) const
    {
        std::size_t seed = std::hash<std::int64_t>{}(key.x);
        seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        return seed;
    }
};

struct Grid2DKey
{
    std::int64_t x;
    std::int64_t y;

    bool operator==(const Grid2DKey& other) const
    {
        return x == other.x && y == other.y;
    }
};

struct Grid2DKeyHash
{
    std::size_t operator()(const Grid2DKey& key) const
    {
        std::size_t seed = std::hash<std::int64_t>{}(key.x);
        seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        return seed;
    }
};
}  // namespace

TrajectoryAndObstaclesPublisher::TrajectoryAndObstaclesPublisher() 
    : Node("scan_planner_interactive_node"),
      has_valid_global_path_(false),
      has_obstacles_(false),
      should_plan_(false),
      needs_replan_(false),
      has_cloud_(false),
      has_odom_(false),
      has_global_path_(false)
{
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", cloud_topic_);
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", odom_topic_);
    global_path_topic_ = this->declare_parameter<std::string>("global_path_topic", global_path_topic_);
    planning_frame_ = this->declare_parameter<std::string>("planning_frame", planning_frame_);
    local_range_x_ = this->declare_parameter<double>("local_range_x", local_range_x_);
    local_range_y_ = this->declare_parameter<double>("local_range_y", local_range_y_);
    local_range_z_down_ = this->declare_parameter<double>("local_range_z_down", local_range_z_down_);
    local_range_z_up_ = this->declare_parameter<double>("local_range_z_up", local_range_z_up_);
    self_filter_x_ = this->declare_parameter<double>("self_filter_x", self_filter_x_);
    self_filter_y_ = this->declare_parameter<double>("self_filter_y", self_filter_y_);
    self_filter_z_ = this->declare_parameter<double>("self_filter_z", self_filter_z_);
    voxel_size_ = this->declare_parameter<double>("voxel_size", voxel_size_);
    max_obstacle_points_ = this->declare_parameter<int>("max_obstacle_points", max_obstacle_points_);
    elevation_grid_resolution_ = this->declare_parameter<double>("elevation_grid_resolution", elevation_grid_resolution_);
    planning_period_ms_ = this->declare_parameter<double>("planning_period_ms", planning_period_ms_);

    local_range_x_ = std::max(0.0, local_range_x_);
    local_range_y_ = std::max(0.0, local_range_y_);
    local_range_z_down_ = std::max(0.0, local_range_z_down_);
    local_range_z_up_ = std::max(0.0, local_range_z_up_);
    self_filter_x_ = std::max(0.0, self_filter_x_);
    self_filter_y_ = std::max(0.0, self_filter_y_);
    self_filter_z_ = std::max(0.0, self_filter_z_);
    voxel_size_ = std::max(1e-3, voxel_size_);
    max_obstacle_points_ = std::max(1, max_obstacle_points_);
    elevation_grid_resolution_ = std::max(1e-3, elevation_grid_resolution_);
    planning_period_ms_ = std::max(20.0, planning_period_ms_);

    // 1. 创建发布者（可视化用）
    global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("visual_global_path", 10);
    // a_star_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("visual_astar_path", 10);
    a_star_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectories", 10);
    local_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("visual_local_trajectory", 10);
    traveled_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("visual_traveled_path", 10);
    obs_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("visual_obstacles", 10);
    elevation_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_elevation_cloud", qos);
    // obs_local_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("visual_local_obstacles", 10);

    inflated_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("inflated_cloud", qos);
    inflated_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("inflated_voxel_marker", qos);
    inflated_edge_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("inflated_voxel_edges", qos);

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&TrajectoryAndObstaclesPublisher::cloud_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&TrajectoryAndObstaclesPublisher::odom_callback, this, std::placeholders::_1));

    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        global_path_topic_,
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::global_path_callback, this, std::placeholders::_1));

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::goal_pose_callback, this, std::placeholders::_1)
    );

    // // 障碍物输入方式1：发布PointCloud2消息
    // rviz_obstacles_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     "/rviz_input_obstacles",
    //     10,
    //     std::bind(&TrajectoryAndObstaclesPublisher::rviz_obstacles_callback, this, std::placeholders::_1)
    // );

    // 路径添加：使用Publish Point工具逐个添加
    rviz_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point",
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::rviz_point_callback, this, std::placeholders::_1)
    );

    // 障碍物输入方式3：使用2D Pose Estimate工具添加
    pose_estimate_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose",
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::pose_estimate_callback, this, std::placeholders::_1)
    );

    // 3. 触发规划的话题
    trigger_plan_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/trigger_plan",
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::trigger_plan_callback, this, std::placeholders::_1)
    );

    // 4. 初始化SCAN-Planner基础配置
    init_scan_planner_base();

    // 5. 定时器：默认 10Hz 触发规划与发布
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(planning_period_ms_)),
        std::bind(&TrajectoryAndObstaclesPublisher::publish_and_plan, this)
    );

    RCLCPP_INFO(this->get_logger(), "Interactive SCAN-Planner Node Ready!");
    RCLCPP_INFO(
        this->get_logger(),
        "World inputs: cloud=%s, odom=%s, global_path=%s, frame=%s",
        cloud_topic_.c_str(), odom_topic_.c_str(), global_path_topic_.c_str(), planning_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "=== 使用方法 ===");
    RCLCPP_INFO(this->get_logger(), "1. 添加全局路径：使用2D Nav Goal工具设置终点");
    RCLCPP_INFO(this->get_logger(), "2. 添加障碍物（三种方法任选）：");
    RCLCPP_INFO(this->get_logger(), "   - 方法A：使用Publish Point工具点击地图");
    RCLCPP_INFO(this->get_logger(), "   - 方法B：使用2D Pose Estimate工具点击Grid任意位置（推荐）");
    RCLCPP_INFO(this->get_logger(), "   - 方法C：发布PointCloud2到 /rviz_input_obstacles");
    RCLCPP_INFO(this->get_logger(), "3. 触发规划：ros2 topic pub /trigger_plan std_msgs/Bool \"{data: true}\"");
    RCLCPP_INFO(this->get_logger(), "4. 停止规划：ros2 topic pub /trigger_plan std_msgs/Bool \"{data: false}\"");
    RCLCPP_INFO(this->get_logger(), "5. 支持多次规划：可以反复发送true/false控制规划");
    RCLCPP_INFO(this->get_logger(), "%s", "");
    RCLCPP_INFO(this->get_logger(), "=== RViz2设置步骤 ===");
    RCLCPP_INFO(this->get_logger(), "1. 添加Grid显示");
    RCLCPP_INFO(this->get_logger(), "2. 添加2D Nav Goal工具（话题：/goal_pose）");
    RCLCPP_INFO(this->get_logger(), "3. 添加2D Pose Estimate工具（使用默认话题：/initialpose）");
    RCLCPP_INFO(this->get_logger(), "4. 添加Publish Point工具（使用默认话题：/clicked_point）");
    RCLCPP_INFO(this->get_logger(), "5. 添加PointCloud2显示（话题：/visual_obstacles）");
    RCLCPP_INFO(this->get_logger(), "6. 添加Path显示（话题：/visual_global_path和/visual_local_trajectory）");
}

// 初始化SCAN-Planner基础参数
void TrajectoryAndObstaclesPublisher::init_scan_planner_base()
{
    scan_planner_ = std::make_shared<PlannerInterface>();
    scan_planner_->initParam(max_vel_, max_acc_, max_jerk_);
    scan_planner_->initEsdfMap(
        map_x_size_, map_y_size_, map_z_size_,
        map_resolution_, map_origin_, map_inflate_value_
    );
}

void TrajectoryAndObstaclesPublisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (msg->header.frame_id != planning_frame_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "忽略里程计: frame_id='%s', 期望 '%s'",
            msg->header.frame_id.c_str(), planning_frame_.c_str());
        return;
    }

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    std::lock_guard<std::mutex> lock(data_mutex_);
    cur_pose_.x = static_cast<float>(msg->pose.pose.position.x);
    cur_pose_.y = static_cast<float>(msg->pose.pose.position.y);
    cur_pose_.z = static_cast<float>(msg->pose.pose.position.z);
    cur_pose_.theta = static_cast<float>(tf2::getYaw(q));
    cur_pose_.v = static_cast<float>(std::hypot(
        msg->twist.twist.linear.x, msg->twist.twist.linear.y));
    has_odom_ = true;
    if (traveled_traj_.empty() || distance(traveled_traj_.back(), cur_pose_) >= traveled_path_min_step_) {
        traveled_traj_.push_back(cur_pose_);
    }
    if (should_plan_ && has_cloud_ && has_global_path_) {
        needs_replan_ = true;
    }
}

void TrajectoryAndObstaclesPublisher::cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (msg->header.frame_id != planning_frame_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "忽略点云: frame_id='%s', 期望 '%s'; 当前阶段不执行 TF 转换",
            msg->header.frame_id.c_str(), planning_frame_.c_str());
        return;
    }

    PathPoint pose_snapshot;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!has_odom_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "已收到点云，但尚无有效里程计，暂不构建局部障碍物");
            return;
        }
        pose_snapshot = cur_pose_;
    }

    std::vector<ObstacleInfo> filtered_obstacles;
    filtered_obstacles.reserve(static_cast<std::size_t>(max_obstacle_points_));
    std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels;
    occupied_voxels.reserve(static_cast<std::size_t>(max_obstacle_points_));

    try {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            const double x = *iter_x;
            const double y = *iter_y;
            const double z = *iter_z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                continue;
            }

            const double dx = x - pose_snapshot.x;
            const double dy = y - pose_snapshot.y;
            const double dz = z - pose_snapshot.z;
            if (std::abs(dx) > local_range_x_ || std::abs(dy) > local_range_y_ ||
                dz < -local_range_z_down_ || dz > local_range_z_up_) {
                continue;
            }
            if (std::abs(dx) <= self_filter_x_ && std::abs(dy) <= self_filter_y_ &&
                std::abs(dz) <= self_filter_z_) {
                continue;
            }

            const VoxelKey key{
                static_cast<std::int64_t>(std::floor(x / voxel_size_)),
                static_cast<std::int64_t>(std::floor(y / voxel_size_)),
                static_cast<std::int64_t>(std::floor(z / voxel_size_))};
            if (!occupied_voxels.insert(key).second) {
                continue;
            }

            filtered_obstacles.push_back(ObstacleInfo{
                static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
            if (static_cast<int>(filtered_obstacles.size()) >= max_obstacle_points_) {
                break;
            }
        }
    } catch (const std::runtime_error& error) {
        RCLCPP_WARN(this->get_logger(), "点云字段解析失败: %s", error.what());
        return;
    }

    const std::size_t obstacle_count = filtered_obstacles.size();
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        obstacles_ = std::move(filtered_obstacles);
        has_cloud_ = true;
        has_obstacles_ = !obstacles_.empty();
        if (should_plan_ && has_global_path_) {
            needs_replan_ = true;
        }
    }

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "局部世界系障碍物: %zu 点, 输入=%u 点", obstacle_count, msg->width * msg->height);
}

void TrajectoryAndObstaclesPublisher::global_path_callback(
    const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->header.frame_id != planning_frame_) {
        RCLCPP_WARN(
            this->get_logger(), "忽略全局路径: frame_id='%s', 期望 '%s'",
            msg->header.frame_id.c_str(), planning_frame_.c_str());
        return;
    }
    if (msg->poses.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "忽略全局路径: 至少需要 2 个路径点");
        return;
    }

    std::vector<PathPoint> path;
    path.reserve(msg->poses.size());
    for (const auto& pose_stamped : msg->poses) {
        tf2::Quaternion q(
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w);
        PathPoint point{};
        point.x = static_cast<float>(pose_stamped.pose.position.x);
        point.y = static_cast<float>(pose_stamped.pose.position.y);
        point.z = static_cast<float>(pose_stamped.pose.position.z);
        point.theta = static_cast<float>(tf2::getYaw(q));
        path.push_back(point);
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        global_plan_traj_ = std::move(path);
        has_global_path_ = true;
        has_valid_global_path_ = true;
        if (should_plan_ && has_cloud_ && has_odom_) {
            needs_replan_ = true;
        }
    }
    RCLCPP_INFO(this->get_logger(), "收到世界系全局路径: %zu 个点", msg->poses.size());
}

// 统一添加障碍物函数
void TrajectoryAndObstaclesPublisher::add_obstacle_at_position(double x, double y)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    ObstacleInfo obs;
    obs.x = x;
    obs.y = y;
    obs.z = cur_pose_.z;
    
    obstacles_.push_back(obs);
    has_obstacles_ = true;
    
    // 如果有障碍物更新且正在规划中，则标记需要重新规划
    if (should_plan_) {
        needs_replan_ = true;
        RCLCPP_INFO(this->get_logger(), "障碍物更新，已标记需要重新规划");
    }
    
    RCLCPP_INFO(this->get_logger(), "添加障碍物: (%.2f, %.2f), 总障碍物数量: %zu", 
                x, y, obstacles_.size());
}

// 2D Pose Estimate回调函数
void TrajectoryAndObstaclesPublisher::pose_estimate_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // In live operation FAST-LIO is authoritative. An RViz 2D Pose Estimate
    // must not overwrite the current odometry and make the path jump.
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (has_odom_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "忽略 /initialpose：已存在 FAST-LIO 实时里程计");
            return;
        }
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
   // 1. 更新机器人初始位姿（原有逻辑）
    cur_pose_.x = msg->pose.pose.position.x;
    cur_pose_.y = msg->pose.pose.position.y;
    cur_pose_.z = msg->pose.pose.position.z;
    // 计算偏航角（使用tf2或手动计算，确保正确）
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    cur_pose_.theta = tf2::getYaw(q);
    std::cout << "pose_estimate_callback " << std::endl;
    // 2. 触发规划逻辑（新增）
    if (has_valid_global_path_) {  // 确保已有全局路径
        needs_replan_ = true;  // 标记需要重新规划
        if (should_plan_) {
            RCLCPP_INFO(this->get_logger(), "初始位置更新，触发重新规划！");
        } else {
            RCLCPP_WARN(this->get_logger(), "初始位置已更新，但规划未启动！请先发送 /trigger_plan true 启动规划");
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "初始位置已更新，但无有效全局路径，无法规划！");
    }
}

// 触发规划话题回调
void TrajectoryAndObstaclesPublisher::trigger_plan_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    should_plan_ = msg->data;
    if (!should_plan_) {
        needs_replan_ = false;
        RCLCPP_INFO(this->get_logger(), "收到 /trigger_plan=false，暂停规划");
        return;
    }

    if (has_odom_ && has_cloud_ && has_global_path_ && global_plan_traj_.size() >= 2) {
        needs_replan_ = true;
        RCLCPP_INFO(this->get_logger(), "收到 /trigger_plan=true，已触发世界系规划");
    } else {
        RCLCPP_WARN(
            this->get_logger(),
            "无法启动规划: odom=%d cloud=%d global_path=%d path_points=%zu",
            has_odom_, has_cloud_, has_global_path_, global_plan_traj_.size());
    }
}

// 处理2D Nav Goal - 生成从起点到目标的直线路
void TrajectoryAndObstaclesPublisher::goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // /goal_pose belongs to the global planner. Do not insert the goal as a
    // local obstacle; global_2d_planner publishes /scan_global_path.
    RCLCPP_INFO(this->get_logger(), "收到全局目标点 (%.2f, %.2f)，由全局规划器处理",
                msg->pose.position.x, msg->pose.position.y);
}

// 生成直线路径
void TrajectoryAndObstaclesPublisher::generate_straight_path(const geometry_msgs::msg::PoseStamped& start, 
                                                           const geometry_msgs::msg::PoseStamped& goal)
{
    global_plan_traj_.clear();

    // 计算路径点数量（每0.1米一个点）
    double dx = goal.pose.position.x - start.pose.position.x;
    double dy = goal.pose.position.y - start.pose.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    int num_points = std::max(2, static_cast<int>(distance / 0.1));

    // 生成直线路径点
    for (int i = 0; i < num_points; ++i) {
        double ratio = static_cast<double>(i) / (num_points - 1);
        PathPoint point;
        point.x = start.pose.position.x + ratio * dx;
        point.y = start.pose.position.y + ratio * dy;
        point.z = 0.2;
        global_plan_traj_.push_back(point);
    }
}

// // 处理Publish Point点击 - 添加障碍物
void TrajectoryAndObstaclesPublisher::rviz_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    // global_plan_traj_.clear();
    PathPoint point;
    point.x = msg->point.x;
    point.y = msg->point.y;
    point.z = msg->point.z;
    point.v = 0.0F;
    point.theta = 0.0F;
    global_plan_traj_.push_back(point);
    std::cout << "从RViz接收到全局路径点: (" << point.x << ", " << point.y << ")" << std::endl;
    has_valid_global_path_ = true;
    has_global_path_ = global_plan_traj_.size() >= 2;
    
    // 如果有路径更新且正在规划中，则标记需要重新规划
    if (should_plan_) 
    {
        needs_replan_ = true;
        RCLCPP_INFO(this->get_logger(), "路径更新，已标记需要重新规划");
    }
}

// 原有的全局路径回调
void TrajectoryAndObstaclesPublisher::rviz_global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    global_path_callback(msg);
}

// 核心逻辑：检查数据更新→触发规划→发布结果
void TrajectoryAndObstaclesPublisher::publish_and_plan()
{
    std::vector<PathPoint> global_path_snapshot;
    std::vector<ObstacleInfo> obstacles_snapshot;
    PathPoint pose_snapshot{};
    bool run_plan = false;

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        run_plan = should_plan_ && needs_replan_ && has_odom_ && has_cloud_ &&
                   has_global_path_ && global_plan_traj_.size() >= 2;
        if (run_plan) {
            global_path_snapshot = global_plan_traj_;
            obstacles_snapshot = obstacles_;
            pose_snapshot = cur_pose_;
            needs_replan_ = false;
        }
    }

    if (run_plan)
    {
        a_star_pathes_.clear();
        std::vector<PathPoint> global_plan_traj_temp;
        discretize_trajectory(global_path_snapshot, global_plan_traj_temp, 0.2);

        float min_dist = std::numeric_limits<float>::max();
        std::size_t min_index = 0;
        for (std::size_t i = 0; i < global_plan_traj_temp.size(); ++i) {
            const double dist = distance(global_plan_traj_temp[i], pose_snapshot);
            if (dist < min_dist) {
                min_dist = static_cast<float>(dist);
                min_index = i;
            }
        }

        std::vector<PathPoint> local_reference;
        local_reference.reserve(global_plan_traj_temp.size() - min_index + 1);
        local_reference.push_back(pose_snapshot);
        for (std::size_t i = min_index; i < global_plan_traj_temp.size(); ++i) {
            local_reference.push_back(global_plan_traj_temp[i]);
        }
        discretize_trajectory(local_reference, global_plan_traj_temp, 0.2);

        if (global_plan_traj_temp.size() < 4) {
            RCLCPP_WARN(
                this->get_logger(), "离散后的局部参考路径点不足: %zu", global_plan_traj_temp.size());
        } else {
            scan_planner_->setPathPoint(global_plan_traj_temp);
            scan_planner_->setGridMapPos(pose_snapshot);
            scan_planner_->setObstacles(obstacles_snapshot);

            // 规划期间不持有 data_mutex_。
            scan_planner_->makePlan();
            scan_planner_->getAStarPath(a_star_pathes_);

            RCLCPP_INFO(
                this->get_logger(), "规划完成: 参考路径=%zu 点, 局部障碍物=%zu 点",
                global_path_snapshot.size(), obstacles_snapshot.size());
        }
    }

    // 发布所有可视化数据（无论是否更新，保持实时显示）
    publish_global_path();
    publish_traveled_path();
    publish_planned_trajectory();
    publish_obstacles();
    publish_elevation_cloud();
    publish_a_star_path();
    publish_local_obstacles();
}


void TrajectoryAndObstaclesPublisher::makePointCloud2(
    const std::vector<Eigen::Vector3d>& points)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = planning_frame_;
    cloud_msg.header.stamp = this->now();
    cloud_msg.height = 1;
    cloud_msg.width = static_cast<std::uint32_t>(points.size());
    cloud_msg.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    for (const auto& point : points) {
        *iter_x = static_cast<float>(point.x());
        *iter_y = static_cast<float>(point.y());
        *iter_z = static_cast<float>(point.z());
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    inflated_cloud_pub_->publish(cloud_msg);
}

void TrajectoryAndObstaclesPublisher::makeInflatedVoxelMarker(
    const std::vector<Eigen::Vector3d>& points, double voxel_size)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = planning_frame_;
  marker.header.stamp = this->now();
  marker.ns = "inflated_voxels";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.2 * voxel_size;
  marker.scale.y = 1.2 * voxel_size;
  marker.scale.z = 1.2 * voxel_size;
  marker.color.r = 1.0f;
  marker.color.g = 0.25f;
  marker.color.b = 0.02f;
  marker.color.a = 0.25f;
  marker.points.reserve(points.size());

  for (const auto& point : points) {
    geometry_msgs::msg::Point marker_point;
    marker_point.x = point.x();
    marker_point.y = point.y();
    marker_point.z = point.z();
    marker.points.push_back(marker_point);
  }

  inflated_marker_pub_->publish(marker);
}

void TrajectoryAndObstaclesPublisher::makeInflatedVoxelEdgeMarker(
    const std::vector<Eigen::Vector3d>& points, double voxel_size)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_;
    marker.header.stamp = this->now();
    marker.ns = "inflated_voxel_edges";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.015;
    marker.color.r = 0.05f;
    marker.color.g = 0.05f;
    marker.color.b = 0.05f;
    marker.color.a = 1.0f;
    marker.points.reserve(points.size() * 24);

    const double half = 0.5 * voxel_size;
    const Eigen::Vector3d offsets[8] = {
        {-half, -half, -half}, {half, -half, -half}, {half, half, -half}, {-half, half, -half},
        {-half, -half, half},  {half, -half, half},  {half, half, half},  {-half, half, half},
    };
    const int edges[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7},
    };

    auto append_point = [&marker](const Eigen::Vector3d& point) {
        geometry_msgs::msg::Point marker_point;
        marker_point.x = point.x();
        marker_point.y = point.y();
        marker_point.z = point.z();
        marker.points.push_back(marker_point);
    };

    for (const auto& center : points) {
        for (const auto& edge : edges) {
        append_point(center + offsets[edge[0]]);
        append_point(center + offsets[edge[1]]);
        }
    }

    inflated_edge_marker_pub_->publish(marker);
}

// 发布可视化全局路径
void TrajectoryAndObstaclesPublisher::publish_global_path()
{
    std::vector<PathPoint> global_path;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        global_path = global_plan_traj_;
    }

    nav_msgs::msg::Path visual_path;
    visual_path.header.stamp = this->now();
    visual_path.header.frame_id = planning_frame_;

    for (const auto& path_point : global_path)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = visual_path.header;
        pose.pose.position.x = path_point.x;
        pose.pose.position.y = path_point.y;
        pose.pose.position.z = path_point.z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        visual_path.poses.push_back(pose);
    }

    global_path_pub_->publish(visual_path);
}

void TrajectoryAndObstaclesPublisher::publish_traveled_path()
{
    std::vector<PathPoint> traveled_path;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        traveled_path = traveled_traj_;
    }

    nav_msgs::msg::Path visual_path;
    visual_path.header.stamp = this->now();
    visual_path.header.frame_id = planning_frame_;

    visual_path.poses.reserve(traveled_path.size());
    for (const auto& path_point : traveled_path)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = visual_path.header;
        pose.pose.position.x = path_point.x;
        pose.pose.position.y = path_point.y;
        pose.pose.position.z = path_point.z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, path_point.theta);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        visual_path.poses.push_back(pose);
    }

    traveled_path_pub_->publish(visual_path);
}

void TrajectoryAndObstaclesPublisher::publish_a_star_path()
{
    // 如果没有路径数据，直接返回
    if (a_star_pathes_.empty())
    {
      RCLCPP_WARN(this->get_logger(), "a_star_pathes_ is empty, no trajectories to publish.");
      return;
    }

    // 创建一个MarkerArray消息
    visualization_msgs::msg::MarkerArray marker_array_msg;

    // 遍历 a_star_pathes_ 中的每一条路径
    for (size_t i = 0; i < a_star_pathes_.size(); ++i)
    {
      const std::vector<Eigen::Vector3d>& current_path = a_star_pathes_[i];

      // 1. 创建一个Marker对象
      visualization_msgs::msg::Marker marker;

      // 2. 设置Marker的基本信息
      marker.header.frame_id = planning_frame_;
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "a_star_paths"; // 命名空间，用于分组管理Marker
      marker.id = i;              // 每条路径必须有唯一的ID
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP; // 类型为线串
      marker.action = visualization_msgs::msg::Marker::ADD;      // 动作是添加

      // 3. 设置线的视觉属性
      marker.scale.x = 0.1; // 线宽为0.1米

      // 设置颜色（RGBA格式），每条路径可以设置不同的颜色
      marker.color.r = 1.0 - (i * 0.3); // 红色分量
      marker.color.g = 0.2;             // 绿色分量
      marker.color.b = 0.2 + (i * 0.3); // 蓝色分量
      marker.color.a = 1.0;             // 透明度（1.0为完全不透明）

      // 4. 填充路径点
      for (const auto& eigen_point : current_path)
      {
        geometry_msgs::msg::Point ros_point;
        ros_point.x = eigen_point.x();
        ros_point.y = eigen_point.y();
        ros_point.z = eigen_point.z(); // 2D轨迹，z轴设为0

        marker.points.push_back(ros_point);
      }

      // 5. 将当前路径的Marker添加到MarkerArray中
      marker_array_msg.markers.push_back(marker);
    }

    // 6. 发布MarkerArray消息
    a_star_path_pub_->publish(marker_array_msg);
    // RCLCPP_INFO(this->get_logger(), "a_star_path_pub_ Published %zu trajectories.", a_star_pathes_.size());

}

// 发布SCAN-Planner规划后的局部轨迹
void TrajectoryAndObstaclesPublisher::publish_planned_trajectory()
{
    planned_traj.clear();
    scan_planner_->getLocalPlanTrajResults(planned_traj);

    // if (planned_traj.empty()) return;

    nav_msgs::msg::Path visual_traj;
    visual_traj.header.stamp = this->now();
    visual_traj.header.frame_id = planning_frame_;

    for (size_t i = 0; i < planned_traj.size(); ++i)
    {
        // std::cout << "[publish_planned_trajectory] x = " << planned_traj[i].x << " y =" << planned_traj[i].y << std::endl;
        geometry_msgs::msg::PoseStamped pose;
        pose.header = visual_traj.header;
        pose.pose.position.x = planned_traj[i].x;
        pose.pose.position.y = planned_traj[i].y;
        pose.pose.position.z = planned_traj[i].z;
        
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        visual_traj.poses.push_back(pose);
    }

    local_traj_pub_->publish(visual_traj);
}

// 发布可视化障碍物
void TrajectoryAndObstaclesPublisher::publish_obstacles()
{
    std::vector<ObstacleInfo> obstacles;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        obstacles = obstacles_;
    }

    sensor_msgs::msg::PointCloud2 visual_obs;
    visual_obs.header.stamp = this->now();
    visual_obs.header.frame_id = planning_frame_;
    visual_obs.width = static_cast<std::uint32_t>(obstacles.size());
    visual_obs.height = 1;
    visual_obs.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(visual_obs);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(obstacles.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(visual_obs, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(visual_obs, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(visual_obs, "z");

    for (const auto& obs : obstacles)
    {
        *iter_x = static_cast<float>(obs.x);
        *iter_y = static_cast<float>(obs.y);
        *iter_z = static_cast<float>(obs.z);
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    obs_pub_->publish(visual_obs);
}

void TrajectoryAndObstaclesPublisher::publish_elevation_cloud()
{
    std::vector<ObstacleInfo> obstacles;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        obstacles = obstacles_;
    }

    std::unordered_map<Grid2DKey, ObstacleInfo, Grid2DKeyHash> highest_cells;
    highest_cells.reserve(obstacles.size());

    for (const auto& obs : obstacles) {
        const Grid2DKey key{
            static_cast<std::int64_t>(std::floor(obs.x / elevation_grid_resolution_)),
            static_cast<std::int64_t>(std::floor(obs.y / elevation_grid_resolution_))};
        auto iter = highest_cells.find(key);
        if (iter == highest_cells.end() || obs.z > iter->second.z) {
            highest_cells[key] = obs;
        }
    }

    sensor_msgs::msg::PointCloud2 elevation_cloud;
    elevation_cloud.header.stamp = this->now();
    elevation_cloud.header.frame_id = planning_frame_;
    elevation_cloud.width = static_cast<std::uint32_t>(highest_cells.size());
    elevation_cloud.height = 1;
    elevation_cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(elevation_cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(highest_cells.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(elevation_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(elevation_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(elevation_cloud, "z");

    for (const auto& cell : highest_cells)
    {
        *iter_x = static_cast<float>((cell.first.x + 0.5) * elevation_grid_resolution_);
        *iter_y = static_cast<float>((cell.first.y + 0.5) * elevation_grid_resolution_);
        *iter_z = static_cast<float>(cell.second.z);
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    elevation_cloud_pub_->publish(elevation_cloud);
}

void TrajectoryAndObstaclesPublisher::publish_local_obstacles()
{
    std::vector<Eigen::Vector3d> obstacles;
    scan_planner_->getObstacles(obstacles);

    makePointCloud2(obstacles);

    makeInflatedVoxelMarker(obstacles, map_resolution_);

    makeInflatedVoxelEdgeMarker(obstacles, map_resolution_);
}

// 计算两点之间的欧氏距离（单位：米）
double TrajectoryAndObstaclesPublisher::distance(const PathPoint& p1, const PathPoint& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

/**
 * 将轨迹离散为均匀间隔的点（间隔10cm）
 * @param original_trajectory 原始轨迹（由多个顶点组成的折线）
 * @param discrete_trajectory 输出的离散轨迹
 * @param interval 间隔距离（单位：米，默认0.1米即10cm）
 */
void TrajectoryAndObstaclesPublisher::discretize_trajectory(const std::vector<PathPoint>& original_trajectory,
                                                            std::vector<PathPoint>& discrete_trajectory,
                                                            double interval) {
    if (original_trajectory.size() < 2) {
        std::cerr << "原始轨迹至少需要2个点！" << std::endl;
        return;
    }

    discrete_trajectory.clear();
    // 添加轨迹起点
    // discrete_trajectory.push_back(cur_pose_);
    discrete_trajectory.push_back(original_trajectory[0]);

    // 遍历原始轨迹的每一段线段
    for (size_t i = 0; i < original_trajectory.size() - 1; ++i) {
        const PathPoint& start = original_trajectory[i];
        const PathPoint& end = original_trajectory[i+1];
        double seg_length = distance(start, end);  // 线段总长度

        if (seg_length < 1e-6) {  // 跳过长度接近0的线段（避免除零）
            continue;
        }

        // 计算当前线段需要插入的点数（不含起点，含终点）。
        int num_points = std::max(1, static_cast<int>(std::ceil(seg_length / interval)));

        // 生成线段上的离散点
        for (int j = 1; j <= num_points; ++j) {
            double ratio;
            if (j < num_points) {
                // 前num_points-1个点：按均匀间隔计算
                ratio = (j * interval) / seg_length;
            } else {
                // 最后一个点：直接对齐到线段终点（避免累积误差）
                ratio = 1.0;
            }

            // 线性插值计算点坐标
            PathPoint p;
            p.x = start.x + ratio * (end.x - start.x);
            p.y = start.y + ratio * (end.y - start.y);
            p.z = start.z + ratio * (end.z - start.z);
            p.v = start.v + ratio * (end.v - start.v);
            p.theta = start.theta + ratio * (end.theta - start.theta);
            discrete_trajectory.push_back(p);
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryAndObstaclesPublisher>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
