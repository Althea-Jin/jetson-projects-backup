#include "plan_env/grid_map.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace plan_env {

GridMap::GridMap(const GridMapConfig& config) { init(config); }

void GridMap::init(const GridMapConfig& config) {
  if (config.resolution <= 0.0) {
    throw std::invalid_argument("GridMap resolution must be positive");
  }
  if ((config.local_map_size.array() <= 0.0).any()) {
    throw std::invalid_argument("GridMap local_map_size must be positive on all axes");
  }

  config_ = config;

  // 使用 ceil，保证实际分配的地图范围至少覆盖 local_map_size。
  for (int i = 0; i < 3; ++i) {
    voxel_num_(i) = static_cast<int>(std::ceil(config_.local_map_size(i) / config_.resolution));
  }

  const std::size_t buffer_size = static_cast<std::size_t>(voxel_num_(0)) *
                                  static_cast<std::size_t>(voxel_num_(1)) *
                                  static_cast<std::size_t>(voxel_num_(2));
  raw_buffer_.assign(buffer_size, 0);
  inflated_buffer_.assign(buffer_size, 0);
  raw_cloud_.clear();
  inflated_cloud_.clear();
  rebuildInflationOffsets();
}

void GridMap::reset() {
  std::fill(raw_buffer_.begin(), raw_buffer_.end(), 0);
  std::fill(inflated_buffer_.begin(), inflated_buffer_.end(), 0);
  raw_cloud_.clear();
  inflated_cloud_.clear();
}

void GridMap::updateLocalMap(const Eigen::Vector3d& current_position,
                             const std::vector<Eigen::Vector3d>& global_cloud) {
  reset();

  // Keep the local volume centered on the robot in all three axes. Point-cloud
  // filtering is relative to the robot, so a fixed world Z origin would drop
  // low obstacles whenever their corrected scan_map height is below zero.
  origin_ = current_position - 0.5 * config_.local_map_size;
  max_boundary_ = origin_ + voxel_num_.cast<double>() * config_.resolution;

  // 输入点云被认为是准确的：不滤波、不概率更新、不做射线投射。
  for (const auto& point : global_cloud) {
    if (!isInMap(point)) {
      continue;
    }
    addOccupiedVoxel(posToIndex(point));
  }
}

GridMap::CellState GridMap::queryRaw(const Eigen::Vector3d& world_position) const {
  return queryBuffer(world_position, raw_buffer_);
}

GridMap::CellState GridMap::queryInflated(const Eigen::Vector3d& world_position) const {
  return queryBuffer(world_position, inflated_buffer_);
}

GridMap::CellState GridMap::queryDoubleCylinder(const Eigen::Vector3d& world_position, double yaw) const {
  const Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
  const Eigen::Vector3d front = world_position + config_.double_cylinder_offset * heading;
  const Eigen::Vector3d rear = world_position - config_.double_cylinder_offset * heading;

  // 只要前后任一圆柱中心落在膨胀障碍物层内，就认为发生占据。
  const CellState front_state = queryInflated(front);
  if (front_state != CellState::kFree) {
    return front_state;
  }
  return queryInflated(rear);
}

bool GridMap::isRawOccupied(const Eigen::Vector3d& world_position) const {
  return queryRaw(world_position) == CellState::kOccupied;
}

bool GridMap::isInflatedOccupied(const Eigen::Vector3d& world_position) const {
  return queryInflated(world_position) == CellState::kOccupied;
}

bool GridMap::getInflateOccupancy(const Eigen::Vector3d& world_position, double yaw) const {
  return queryDoubleCylinder(world_position, yaw) == CellState::kOccupied;
}

bool GridMap::isInMap(const Eigen::Vector3d& world_position) const {
  return world_position.x() >= origin_.x() && world_position.y() >= origin_.y() &&
         world_position.z() >= origin_.z() && world_position.x() < max_boundary_.x() &&
         world_position.y() < max_boundary_.y() && world_position.z() < max_boundary_.z();
}

bool GridMap::isInMap(const Eigen::Vector3i& index) const {
  return index.x() >= 0 && index.y() >= 0 && index.z() >= 0 && index.x() < voxel_num_.x() &&
         index.y() < voxel_num_.y() && index.z() < voxel_num_.z();
}

Eigen::Vector3i GridMap::posToIndex(const Eigen::Vector3d& world_position) const {
  return ((world_position - origin_) / config_.resolution).array().floor().cast<int>();
}

Eigen::Vector3d GridMap::indexToPos(const Eigen::Vector3i& index) const {
  return origin_ + (index.cast<double>() + Eigen::Vector3d::Constant(0.5)) * config_.resolution;
}

void GridMap::rebuildInflationOffsets() {
  inflation_offsets_.clear();

  const int step_xy = static_cast<int>(std::ceil(std::max(0.0, config_.inflation_radius) /
                                                config_.resolution));
  const int step_z_up = static_cast<int>(std::ceil(std::max(0.0, config_.inflation_z_up) /
                                                  config_.resolution));
  const int step_z_down = static_cast<int>(std::ceil(std::max(0.0, config_.inflation_z_down) /
                                                    config_.resolution));

  // 预计算一个竖直圆柱的体素偏移。XY 方向按实际米制半径判断。
  for (int x = -step_xy; x <= step_xy; ++x) {
    for (int y = -step_xy; y <= step_xy; ++y) {
      const double dx = static_cast<double>(x) * config_.resolution;
      const double dy = static_cast<double>(y) * config_.resolution;
      if (std::hypot(dx, dy) > config_.inflation_radius) {
        continue;
      }
      for (int z = -step_z_down; z <= step_z_up; ++z) {
        inflation_offsets_.emplace_back(x, y, z);
      }
    }
  }
}

void GridMap::addOccupiedVoxel(const Eigen::Vector3i& index) {
  if (!isInMap(index)) {
    return;
  }

  const std::size_t address = toAddress(index);
  if (!raw_buffer_[address]) {
    raw_buffer_[address] = 1;
    raw_cloud_.push_back(indexToPos(index));
  }
  addInflationAround(index);
}

void GridMap::addInflationAround(const Eigen::Vector3i& index) {
  for (const auto& offset : inflation_offsets_) {
    const Eigen::Vector3i inflated_index = index + offset;
    if (!isInMap(inflated_index)) {
      continue;
    }

    const std::size_t address = toAddress(inflated_index);
    if (!inflated_buffer_[address]) {
      inflated_buffer_[address] = 1;
      inflated_cloud_.push_back(indexToPos(inflated_index));
    }
  }
}

std::size_t GridMap::toAddress(const Eigen::Vector3i& index) const {
  return (static_cast<std::size_t>(index.x()) * static_cast<std::size_t>(voxel_num_.y()) +
          static_cast<std::size_t>(index.y())) *
             static_cast<std::size_t>(voxel_num_.z()) +
         static_cast<std::size_t>(index.z());
}

GridMap::CellState GridMap::queryBuffer(const Eigen::Vector3d& world_position,
                                        const std::vector<unsigned char>& buffer) const {
  if (!isInMap(world_position)) {
    return CellState::kOutOfMap;
  }
  return buffer[toAddress(posToIndex(world_position))] ? CellState::kOccupied : CellState::kFree;
}

}  // namespace plan_env
