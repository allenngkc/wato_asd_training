#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initializeCostmap(double width, double height, double resolution);
    void markObstacle(int x, int y);
    void inflateObstacle(int x, int y, double radius, double max_cost);
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);

    std::vector<std::vector<int8_t>> getOccupancyGrid() const;
    double getWidth() const;
    double getHeight() const;
    double getResolution() const;

    void setOccupancyGrid(int x, int y, int8_t value) {
      if (x < 0 || x >= width_ || y < 0 || y >= height_) {
          RCLCPP_WARN(logger_, "Attempted to set occupancy grid out of bounds");
          return;
      }
      occupancy_grid_[x][y] = value;
    }

  private:
    rclcpp::Logger logger_;
    std::vector<std::vector<int8_t>> occupancy_grid_;
    double resolution_;
    double width_;
    double height_;
};

}  

#endif  