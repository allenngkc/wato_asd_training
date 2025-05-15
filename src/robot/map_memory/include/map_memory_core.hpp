#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <cmath>

namespace robot
{

class MapMemoryCore
{
public:
  MapMemoryCore(const rclcpp::Logger& logger);

  // Global map and robot position
  nav_msgs::msg::OccupancyGrid global_map_;
  double last_x;
  double last_y;
  double last_yaw;
  const double distance_threshold;

  // Integrate the latest costmap into the global map
  void integrateCostmap();

  // Flags
  nav_msgs::msg::OccupancyGrid latest_costmap_;

private:
  rclcpp::Logger logger_;
};

}
#endif  