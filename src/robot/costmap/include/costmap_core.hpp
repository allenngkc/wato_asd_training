#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initializeCostmap(
      double resolution,
      int width,
      int height,
      const geometry_msgs::msg::Pose origin,
      double inflation_radius
    );
    
    void inflateObstacle(int x, int y) const;
    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan) const;

    std::vector<std::vector<int8_t>> getOccupancyGrid() const;
    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmapData() const;
    void setOccupancyGrid(int x, int y, int8_t value) const;

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;
    std::vector<std::vector<int8_t>> occupancy_grid_;
    double resolution_;
    double width_;
    double height_;
    double inflation_radius_;
    int inflation_cells_;
    const int max_cost = 100;
};

}  

#endif  