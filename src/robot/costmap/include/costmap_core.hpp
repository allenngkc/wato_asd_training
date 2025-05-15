
#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void setInput(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    void initializeCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    void convertToGrid(float range, float angle, int &x_grid, int &y_grid);

    void markObstacle(int x_grid, int y_grid);
    
    void inflateObstacles();
    
    nav_msgs::msg::OccupancyGrid::SharedPtr local_map_;  // local occupancy map in robot's lidar frame

  private:
    rclcpp::Logger logger_;

    // map info
    int X_, Y_;  // origin's coordinates
    int W_, H_;  // width and height
    double RES_;  // resolution
    double MAX_COST_;
};
}  

#endif  
