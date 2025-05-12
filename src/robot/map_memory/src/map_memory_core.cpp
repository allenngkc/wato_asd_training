#include "map_memory_core.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <eigen3/Eigen/Core>

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
}

void MapMemoryCore::integrateCostmap() {
  for (size_t i = 0; i < global_map_.data.size(); i++) {
      int w = global_map_.info.width;
      double res = global_map_.info.resolution;
      double x = (int(i % w) + 0.5) * res;
      double y = (int(i / w) + 0.5) * res;

      Eigen::Vector2d p_local;
      p_local << x + (global_map_.info.origin.position.x - last_x),
                  y + (global_map_.info.origin.position.y - last_y);
      Eigen::Matrix2d rot_mat;
      rot_mat << cos(last_yaw), sin(last_yaw),
                  -sin(last_yaw), cos(last_yaw);
                  
      p_local = rot_mat * p_local;
      double x_local = p_local[0];
      double y_local = p_local[1];

      int x_grid = (x_local - latest_costmap_.info.origin.position.x) / latest_costmap_.info.resolution;
      int y_grid = (y_local - latest_costmap_.info.origin.position.y) / latest_costmap_.info.resolution;

      if (x_grid < latest_costmap_.info.width && y_grid < latest_costmap_.info.height && x_grid > 0 && y_grid > 0) {
          size_t idx_local = y_grid * latest_costmap_.info.width + x_grid;

          int8_t cost = latest_costmap_.data.at(idx_local);
          if (cost > 0)
          global_map_.data.at(i) = std::max(global_map_.data.at(i), cost);
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
