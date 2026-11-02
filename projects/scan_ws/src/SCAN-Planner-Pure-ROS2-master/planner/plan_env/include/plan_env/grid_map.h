/**
 * @file      grid_map.h
 * @brief     SCAN-Planner robo-centric local occupancy grid map.
 * @author    juchunyu <juchunyu@qq.com>
 * @date      2026-07-23 20:00:01
 * @copyright Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC).
 *            All rights reserved.
 * This library provides real-time occupancy queries for navigation and collision avoidance.
 */

#ifndef PLAN_ENV_GRID_MAP_H_
#define PLAN_ENV_GRID_MAP_H_

#include <Eigen/Core>

#include <cstddef>
#include <memory>
#include <vector>

namespace plan_env {

// 纯 C++ 局部占据地图配置。所有距离单位都是米。
struct GridMapConfig {
  // 体素边长。
  double resolution = 0.2;

  // 以机器人当前位置为中心的局部地图尺寸。
  Eigen::Vector3d local_map_size = Eigen::Vector3d(20.0, 20.0, 5.0);

  // 单个障碍物膨胀圆柱：XY 半径和竖直方向范围。
  double inflation_radius = 0.5;
  double inflation_z_up = 0.5;
  double inflation_z_down = 0.2;

  // 双圆柱查询时，沿查询 yaw 方向前后偏移的距离。
  // 设为 0.0 时等价于只查询位于查询点中心的单圆柱。
  double double_cylinder_offset = 0.0;
};

class GridMap {
public:
  using Ptr = std::shared_ptr<GridMap>;

  // 查询结果：局部地图外、空闲体素、占据体素。
  enum class CellState { kOutOfMap = -1, kFree = 0, kOccupied = 1 };

  GridMap() = default;
  explicit GridMap(const GridMapConfig& config);

  // 分配地图缓存，并预计算膨胀体素偏移。
  void init(const GridMapConfig& config);

  // 清空原始占据层和膨胀占据层，保留配置不变。
  void reset();

  // 使用准确的全局点云坐标构建局部地图。
  // current_position 用来确定局部地图 XY 中心；global_cloud 已经在世界坐标系下。
  void updateLocalMap(const Eigen::Vector3d& current_position,
                      const std::vector<Eigen::Vector3d>& global_cloud);

  // 查询由输入点云直接生成的原始占据体素。
  CellState queryRaw(const Eigen::Vector3d& world_position) const;

  // 查询障碍物膨胀层。
  CellState queryInflated(const Eigen::Vector3d& world_position) const;

  // 查询沿 yaw 方向前后偏移的两个膨胀圆柱。
  CellState queryDoubleCylinder(const Eigen::Vector3d& world_position, double yaw) const;

  bool isRawOccupied(const Eigen::Vector3d& world_position) const;
  bool isInflatedOccupied(const Eigen::Vector3d& world_position) const;
  bool getInflateOccupancy(const Eigen::Vector3d& world_position, double yaw) const;

  bool isInMap(const Eigen::Vector3d& world_position) const;
  bool isInMap(const Eigen::Vector3i& index) const;

  // 世界坐标和局部体素索引之间的转换。
  Eigen::Vector3i posToIndex(const Eigen::Vector3d& world_position) const;
  Eigen::Vector3d indexToPos(const Eigen::Vector3i& index) const;

  const Eigen::Vector3d& origin() const { return origin_; }
  const Eigen::Vector3d& size() const { return config_.local_map_size; }
  const Eigen::Vector3i& voxelNum() const { return voxel_num_; }
  double resolution() const { return config_.resolution; }
  double getResolution() const { return config_.resolution; }

  // 体素中心点云，可用于外部可视化或调试。
  const std::vector<Eigen::Vector3d>& rawCloud() const { return raw_cloud_; }
  const std::vector<Eigen::Vector3d>& inflatedCloud() const { return inflated_cloud_; }

private:
  void rebuildInflationOffsets();
  void addOccupiedVoxel(const Eigen::Vector3i& index);
  void addInflationAround(const Eigen::Vector3i& index);
  std::size_t toAddress(const Eigen::Vector3i& index) const;
  CellState queryBuffer(const Eigen::Vector3d& world_position, const std::vector<unsigned char>& buffer) const;

  GridMapConfig config_;
  Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d max_boundary_ = Eigen::Vector3d::Zero();
  Eigen::Vector3i voxel_num_ = Eigen::Vector3i::Zero();

  // 缓存按局部体素顺序存储 0/1 占据状态。
  std::vector<unsigned char> raw_buffer_;
  std::vector<unsigned char> inflated_buffer_;
  std::vector<Eigen::Vector3i> inflation_offsets_;
  std::vector<Eigen::Vector3d> raw_cloud_;
  std::vector<Eigen::Vector3d> inflated_cloud_;
};

}  // namespace plan_env

#endif  // PLAN_ENV_GRID_MAP_H_
