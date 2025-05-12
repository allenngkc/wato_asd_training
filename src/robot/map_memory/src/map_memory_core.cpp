#include "map_memory_core.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace robot
{

void MapMemoryCore::initMapMemory(double resolution, int width, int height, const geometry_msgs::msg::Pose origin) {
  global_map_.info.resolution = resolution;
  global_map_.info.width = width;
  global_map_.info.height = height;
  global_map_.info.origin = origin;
  global_map_.data.resize(global_map_.info.width * global_map_.info.height, -1);
  global_map_.data.assign(width * height, 0);
}

void MapMemoryCore::setCostmapUpdated(bool val) {
  costmap_updated_ = val;
}
void MapMemoryCore::setlatestCostmap(nav_msgs::msg::OccupancyGrid msg) {
  latest_costmap_ = msg;
}
void MapMemoryCore::setShouldUpdateMap(bool val) {
  should_update_map_ = val;
}
void MapMemoryCore::setlatestOdom(nav_msgs::msg::Odometry msg) {
  latest_odom_ = msg;
  last_x = latest_odom_.pose.pose.position.x;
  last_y = latest_odom_.pose.pose.position.y;
}

void MapMemoryCore::integrateCostmap() {
  const geometry_msgs::msg::Quaternion &q = latest_odom_.pose.pose.orientation;
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 mat(quat);
  double roll,pitch,yaw;
  mat.getRPY(roll,pitch,yaw);
  
  for (size_t j{0}; j < latest_costmap_.info.height; ++j) {
    for (size_t i{0}; i < latest_costmap_.info.width; ++i) {
      int local_index = j * latest_costmap_.info.width + i;

      int8_t cost = latest_costmap_.data[local_index];
      if (cost == -1 || cost == 0) {
        continue;
      }
      double local_x = (i * latest_costmap_.info.resolution) + latest_costmap_.info.origin.position.x;
      double local_y = (j * latest_costmap_.info.resolution) + latest_costmap_.info.origin.position.y;

      double global_x = last_x + local_x * std::cos(yaw) - local_y * std::sin(yaw);
      double global_y = last_y + local_x * std::sin(yaw) + local_y * std::cos(yaw);

      int global_i = static_cast<int>((global_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
      int global_j = static_cast<int>((global_y - global_map_.info.origin.position.y) / global_map_.info.resolution);

      if (global_i < 0 || global_i >= static_cast<int>(global_map_.info.width) || global_j < 0 || global_j >= static_cast<int>(global_map_.info.height)) {
        continue;
      }

      int global_index = global_j * global_map_.info.width + global_i;

      if (global_map_.data[global_index] == -1 || cost > global_map_.data[global_index]) {
        global_map_.data[global_index] = cost;
      }
    }
  }

}

void MapMemoryCore::setLastX(double x) { last_x = x; }
void MapMemoryCore::setLastY(double y) { last_y = y; }

double MapMemoryCore::getLastX()  { return last_x; }
double MapMemoryCore::getLastY()  { return last_y; }
double MapMemoryCore::getDistanceThreshold()  { return distance_threshold; }
nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap()  { return global_map_; }
bool MapMemoryCore::getCostmapUpdate()  { return costmap_updated_; }
bool MapMemoryCore::getShouldUpdateMap()  { return should_update_map_; }

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

} 
