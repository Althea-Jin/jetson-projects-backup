// Build a conservative 2.5D traversability map from FAST-LIO's accumulated
// world cloud and publish an A* path for SCAN-Planner.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

struct CellKey
{
  std::int64_t x;
  std::int64_t y;
  bool operator==(const CellKey & other) const { return x == other.x && y == other.y; }
};

struct CellHash
{
  std::size_t operator()(const CellKey & key) const
  {
    auto h1 = std::hash<std::int64_t>{}(key.x);
    auto h2 = std::hash<std::int64_t>{}(key.y);
    return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6U) + (h1 >> 2U));
  }
};

struct CellData
{
  float min_z{std::numeric_limits<float>::max()};
  float max_z{std::numeric_limits<float>::lowest()};
  double last_seen_seconds{0.0};
};

struct GridIndex
{
  int x;
  int y;
  bool operator==(const GridIndex & other) const { return x == other.x && y == other.y; }
};

struct GridHash
{
  std::size_t operator()(const GridIndex & key) const
  {
    return std::hash<int>{}(key.x) ^ (std::hash<int>{}(key.y) << 1U);
  }
};

class Global2DPlanner : public rclcpp::Node
{
public:
  Global2DPlanner()
  : Node("global_2d_planner")
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/scan/cloud_registered");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/scan/Odometry");
    planning_frame_ = declare_parameter<std::string>("planning_frame", "scan_map");
    resolution_ = declare_parameter<double>("resolution", 0.20);
    obstacle_height_ = declare_parameter<double>("obstacle_height", 0.25);
    inflation_radius_ = declare_parameter<double>("inflation_radius", 0.20);
    free_space_fill_radius_ = declare_parameter<double>("free_space_fill_radius", 0.30);
    endpoint_search_radius_ = declare_parameter<double>("endpoint_search_radius", 1.0);
    cell_timeout_ = declare_parameter<double>("cell_timeout", 8.0);
    unknown_is_occupied_ = declare_parameter<bool>("unknown_is_occupied", true);
    map_update_period_ = declare_parameter<double>("map_update_period", 1.0);
    resolution_ = std::max(0.03, resolution_);
    obstacle_height_ = std::max(0.02, obstacle_height_);
    inflation_radius_ = std::max(0.0, inflation_radius_);
    free_space_fill_radius_ = std::max(0.0, free_space_fill_radius_);
    endpoint_search_radius_ = std::max(0.0, endpoint_search_radius_);
    cell_timeout_ = std::max(0.0, cell_timeout_);
    map_update_period_ = std::max(0.1, map_update_period_);

    map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      map_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Global2DPlanner::map_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Global2DPlanner::odom_callback, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&Global2DPlanner::goal_callback, this, std::placeholders::_1));

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    elevation_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("global_elevation_cloud", map_qos);
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("global_costmap", map_qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/scan_global_path", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    RCLCPP_INFO(get_logger(), "Global 2.5D planner: map=%s odom=%s frame=%s resolution=%.2f",
      map_topic_.c_str(), odom_topic_.c_str(), planning_frame_.c_str(), resolution_);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg->header.frame_id != planning_frame_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    start_x_ = msg->pose.pose.position.x;
    start_y_ = msg->pose.pose.position.y;
    start_z_ = msg->pose.pose.position.z;
    have_start_ = true;
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != planning_frame_) {
      RCLCPP_WARN(get_logger(), "Ignore goal frame '%s', expected '%s'",
        msg->header.frame_id.c_str(), planning_frame_.c_str());
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    goal_z_ = msg->pose.position.z;
    have_goal_ = true;
    if (have_start_ && !cells_.empty()) plan_locked();
  }

  void map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->header.frame_id != planning_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Ignore map frame '%s', expected '%s'", msg->header.frame_id.c_str(), planning_frame_.c_str());
      return;
    }

    const double update_time = now().seconds();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (update_time - last_map_update_seconds_ < map_update_period_) return;
      last_map_update_seconds_ = update_time;
    }

    std::unordered_map<CellKey, CellData, CellHash> next;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
        const CellKey key{static_cast<std::int64_t>(std::floor(*x / resolution_)),
                          static_cast<std::int64_t>(std::floor(*y / resolution_))};
        auto & cell = next[key];
        cell.min_z = std::min(cell.min_z, *z);
        cell.max_z = std::max(cell.max_z, *z);
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN(get_logger(), "Global map fields unavailable: %s", error.what());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      // /scan/cloud_registered is already expressed in the fixed world frame.
      // Refresh cells from the latest observation. A cell that is not seen is
      // retained only for cell_timeout_; this removes dynamic obstacles after
      // they leave instead of permanently accumulating their height range.
      for (const auto & entry : next) {
        auto refreshed = entry.second;
        refreshed.last_seen_seconds = update_time;
        cells_[entry.first] = refreshed;
      }
      if (cell_timeout_ > 0.0) {
        for (auto iter = cells_.begin(); iter != cells_.end();) {
          if (update_time - iter->second.last_seen_seconds > cell_timeout_) {
            iter = cells_.erase(iter);
          } else {
            ++iter;
          }
        }
      }
      publish_maps_locked();
      if (have_start_ && have_goal_) plan_locked();
    }
  }

  bool is_blocked(const CellData & cell) const
  {
    return (cell.max_z - cell.min_z) >= obstacle_height_;
  }

  void publish_maps_locked()
  {
    if (cells_.empty()) {
      nav_msgs::msg::OccupancyGrid empty_grid;
      empty_grid.header.stamp = now();
      empty_grid.header.frame_id = planning_frame_;
      empty_grid.info.resolution = resolution_;
      sensor_msgs::msg::PointCloud2 empty_elevation;
      empty_elevation.header = empty_grid.header;
      empty_elevation.height = 1;
      empty_elevation.width = 0;
      empty_elevation.is_dense = true;
      sensor_msgs::PointCloud2Modifier empty_modifier(empty_elevation);
      empty_modifier.setPointCloud2FieldsByString(1, "xyz");
      empty_modifier.resize(0);
      elevation_pub_->publish(empty_elevation);
      costmap_pub_->publish(empty_grid);
      map_width_ = map_height_ = 0;
      costmap_.clear();
      return;
    }
    std::int64_t min_x = std::numeric_limits<std::int64_t>::max();
    std::int64_t min_y = std::numeric_limits<std::int64_t>::max();
    std::int64_t max_x = std::numeric_limits<std::int64_t>::lowest();
    std::int64_t max_y = std::numeric_limits<std::int64_t>::lowest();
    for (const auto & entry : cells_) {
      min_x = std::min(min_x, entry.first.x); max_x = std::max(max_x, entry.first.x);
      min_y = std::min(min_y, entry.first.y); max_y = std::max(max_y, entry.first.y);
    }
    const int width = static_cast<int>(max_x - min_x + 1);
    const int height = static_cast<int>(max_y - min_y + 1);
    if (width <= 0 || height <= 0 || static_cast<std::size_t>(width) * height > 4000000U) return;

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = planning_frame_;
    grid.info.resolution = resolution_;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position.x = min_x * resolution_;
    grid.info.origin.position.y = min_y * resolution_;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<std::size_t>(width) * height, -1);

    sensor_msgs::msg::PointCloud2 elevation;
    elevation.header = grid.header;
    elevation.height = 1;
    elevation.width = static_cast<std::uint32_t>(cells_.size());
    elevation.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(elevation);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(cells_.size());
    sensor_msgs::PointCloud2Iterator<float> ex(elevation, "x");
    sensor_msgs::PointCloud2Iterator<float> ey(elevation, "y");
    sensor_msgs::PointCloud2Iterator<float> ez(elevation, "z");

    for (const auto & entry : cells_) {
      const int ix = static_cast<int>(entry.first.x - min_x);
      const int iy = static_cast<int>(entry.first.y - min_y);
      const auto index = static_cast<std::size_t>(iy) * width + ix;
      grid.data[index] = is_blocked(entry.second) ? 100 : 0;
      *ex = static_cast<float>((entry.first.x + 0.5) * resolution_);
      *ey = static_cast<float>((entry.first.y + 0.5) * resolution_);
      *ez = entry.second.min_z;
      ++ex; ++ey; ++ez;
    }

    // Registered MID-360 ground returns are not dense enough to touch every
    // 2D cell. Bridge only small gaps around directly observed free cells so
    // unknown_is_occupied can remain conservative for genuinely unobserved
    // regions. Direct obstacles are never overwritten, and obstacle inflation
    // below is applied after this fill step.
    const int fill_cells = static_cast<int>(std::ceil(free_space_fill_radius_ / resolution_));
    if (fill_cells > 0) {
      const auto directly_observed = grid.data;
      for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
        if (directly_observed[static_cast<std::size_t>(y) * width + x] != 0) continue;
        for (int dy = -fill_cells; dy <= fill_cells; ++dy) {
          for (int dx = -fill_cells; dx <= fill_cells; ++dx) {
            if (std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * resolution_ >
              free_space_fill_radius_) continue;
            const int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            const auto index = static_cast<std::size_t>(ny) * width + nx;
            if (directly_observed[index] < 0) grid.data[index] = 0;
          }
        }
      }
    }

    const int inflate = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
    if (inflate > 0) {
      auto original = grid.data;
      for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
        if (original[static_cast<std::size_t>(y) * width + x] != 100) continue;
        for (int dy = -inflate; dy <= inflate; ++dy) for (int dx = -inflate; dx <= inflate; ++dx) {
          if (dx * dx + dy * dy > inflate * inflate) continue;
          const int nx = x + dx, ny = y + dy;
          if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
            grid.data[static_cast<std::size_t>(ny) * width + nx] = 100;
          }
        }
      }
    }
    elevation_pub_->publish(elevation);
    costmap_pub_->publish(grid);
    map_min_x_ = min_x; map_min_y_ = min_y; map_width_ = width; map_height_ = height;
    costmap_ = std::move(grid.data);
  }

  void plan_locked()
  {
    if (map_width_ <= 0 || map_height_ <= 0 || costmap_.empty()) return;
    auto to_grid = [this](double x, double y) {
      return GridIndex{static_cast<int>(std::floor(x / resolution_)) - static_cast<int>(map_min_x_),
                       static_cast<int>(std::floor(y / resolution_)) - static_cast<int>(map_min_y_)};
    };
    const GridIndex requested_start = to_grid(start_x_, start_y_);
    const GridIndex requested_goal = to_grid(goal_x_, goal_y_);
    auto valid = [this](const GridIndex & p) {
      return p.x >= 0 && p.x < map_width_ && p.y >= 0 && p.y < map_height_;
    };
    if (!valid(requested_start) || !valid(requested_goal)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Start or goal outside 2.5D map");
      return;
    }
    auto offset = [this](const GridIndex & p) { return static_cast<std::size_t>(p.y) * map_width_ + p.x; };
    auto traversable = [this, &offset](const GridIndex & p) {
      const auto state = costmap_[offset(p)];
      return state != 100 && (!unknown_is_occupied_ || state >= 0);
    };
    auto nearest_free = [this, &valid, &traversable](const GridIndex & requested, GridIndex & result) {
      if (traversable(requested)) {
        result = requested;
        return true;
      }
      const int radius = static_cast<int>(std::ceil(endpoint_search_radius_ / resolution_));
      double best_distance = std::numeric_limits<double>::infinity();
      bool found = false;
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          const double distance = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
          if (distance > radius || distance >= best_distance) continue;
          const GridIndex candidate{requested.x + dx, requested.y + dy};
          if (!valid(candidate) || !traversable(candidate)) continue;
          result = candidate;
          best_distance = distance;
          found = true;
        }
      }
      return found;
    };

    GridIndex start, goal;
    if (!nearest_free(requested_start, start)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Start is occupied and no free cell exists within %.2f m", endpoint_search_radius_);
      return;
    }
    if (!nearest_free(requested_goal, goal)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Goal is occupied and no free cell exists within %.2f m", endpoint_search_radius_);
      return;
    }
    if (!(start == requested_start) || !(goal == requested_goal)) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Adjusted occupied endpoint: start_shift=%.2f m goal_shift=%.2f m",
        std::hypot(start.x - requested_start.x, start.y - requested_start.y) * resolution_,
        std::hypot(goal.x - requested_goal.x, goal.y - requested_goal.y) * resolution_);
    }
    struct QueueItem {
      double priority;
      double path_cost;
      GridIndex p;
      bool operator>(const QueueItem & o) const { return priority > o.priority; }
    };
    const int total = map_width_ * map_height_;
    std::vector<double> cost(static_cast<std::size_t>(total), std::numeric_limits<double>::infinity());
    std::vector<GridIndex> parent(static_cast<std::size_t>(total), GridIndex{-1, -1});
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
    cost[offset(start)] = 0.0;
    queue.push({std::hypot(start.x - goal.x, start.y - goal.y), 0.0, start});
    const int dirs[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    while (!queue.empty()) {
      const auto current = queue.top(); queue.pop();
      if (current.p == goal) break;
      if (current.path_cost > cost[offset(current.p)]) continue;
      for (const auto & d : dirs) {
        GridIndex next{current.p.x + d[0], current.p.y + d[1]};
        if (!valid(next)) continue;
        const auto state = costmap_[offset(next)];
        if (state == 100 || (state < 0 && unknown_is_occupied_)) continue;
        const double step = (d[0] != 0 && d[1] != 0) ? 1.41421356 : 1.0;
        const double candidate = current.path_cost + step;
        if (candidate < cost[offset(next)]) {
          cost[offset(next)] = candidate;
          parent[offset(next)] = current.p;
          const double dx = next.x - goal.x, dy = next.y - goal.y;
          queue.push({candidate + std::hypot(dx, dy), candidate, next});
        }
      }
    }
    if (!std::isfinite(cost[offset(goal)])) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "No global path found");
      return;
    }
    std::vector<GridIndex> reverse;
    for (GridIndex p = goal; !(p == GridIndex{-1, -1}); p = parent[offset(p)]) {
      reverse.push_back(p);
      if (p == start) break;
    }
    if (!(reverse.back() == start)) return;
    std::reverse(reverse.begin(), reverse.end());
    nav_msgs::msg::Path path;
    path.header.stamp = now(); path.header.frame_id = planning_frame_;
    for (const auto & p : reverse) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = (p.x + map_min_x_ + 0.5) * resolution_;
      pose.pose.position.y = (p.y + map_min_y_ + 0.5) * resolution_;
      pose.pose.position.z = start_z_;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    if (!path.poses.empty()) path.poses.front().pose.position.z = start_z_;
    if (!path.poses.empty()) path.poses.back().pose.position.z = goal_z_;
    path_pub_->publish(path);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "Published global path: %zu points", path.poses.size());
  }

  std::string map_topic_, odom_topic_, planning_frame_;
  double resolution_{0.2}, obstacle_height_{0.25}, inflation_radius_{0.20};
  double free_space_fill_radius_{0.30};
  double endpoint_search_radius_{1.0};
  double cell_timeout_{8.0};
  double map_update_period_{1.0};
  double last_map_update_seconds_{-std::numeric_limits<double>::infinity()};
  bool unknown_is_occupied_{true};
  std::mutex mutex_;
  std::unordered_map<CellKey, CellData, CellHash> cells_;
  std::vector<int8_t> costmap_;
  int map_width_{0}, map_height_{0};
  std::int64_t map_min_x_{0}, map_min_y_{0};
  double start_x_{0.0}, start_y_{0.0}, start_z_{0.0};
  double goal_x_{0.0}, goal_y_{0.0}, goal_z_{0.0};
  bool have_start_{false}, have_goal_{false};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr elevation_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Global2DPlanner>());
  rclcpp::shutdown();
  return 0;
}
