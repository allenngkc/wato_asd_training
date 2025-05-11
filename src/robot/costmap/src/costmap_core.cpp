#include "costmap_core.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger), costmap_data_(std::make_shared<nav_msgs::msg::OccupancyGrid>()) {}
 
    void CostmapCore::initializeCostmap(
        double resolution,
        int width,
        int height,
        const geometry_msgs::msg::Pose origin,
        double inflation_radius
    ) {
        resolution_ = resolution;
        inflation_radius_ = inflation_radius;
        inflation_cells_ = static_cast<int>(inflation_radius / resolution);
        
        costmap_data_->info.resolution = resolution_;
        costmap_data_->info.width = width;
        costmap_data_->info.height = height;
        costmap_data_->info.origin = origin;
        costmap_data_->data.assign(width * height, 0);
    }

    void CostmapCore::inflateObstacle(int x, int y) const {
    for (int i = -inflation_cells_; i <= inflation_cells_; ++i) {
        for (int j = -inflation_cells_; j <= inflation_cells_; ++j) {
            int nx = x + i;
            int ny = y + j;

            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                double distance = std::sqrt(i*i+j*j) * resolution_;
                if (distance > inflation_cells_) continue;

                double cost = max_cost * (1.0 - distance / inflation_cells_);
                
                if (cost > costmap_data_->data[ny * costmap_data_->info.width + nx]) {
                    costmap_data_->data[ny * costmap_data_->info.width + nx] = static_cast<int8_t>(cost);
                }
            }
        }
    } 
    }

    void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan) const {
        std::fill(costmap_data_->data.begin(), costmap_data_->data.end(), 0);
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            double range = scan->ranges[i];
            double angle = scan->angle_min + i * scan->angle_increment;
            if (range > scan->range_max || range < scan->range_min) continue;

            double x = range * std::cos(angle);
            double y = range * std::sin(angle);

            int gx = static_cast<int>((x - costmap_data_->info.origin.position.x) / resolution_);
            int gy = static_cast<int>((y - costmap_data_->info.origin.position.y) / resolution_);

            if (gx >= 0 && gx < static_cast<int>(costmap_data_->info.width) &&
                gy >= 0 && gy < static_cast<int>(costmap_data_->info.height)) {
                int idx = gy * costmap_data_->info.width + gx;
                costmap_data_->data[idx] = 100;

                inflateObstacle(gx, gy);
            }
        }
    }

    void CostmapCore::setOccupancyGrid(int x, int y, int8_t value) const {
      if (x < 0 || x >= width_ || y < 0 || y >= height_) {
          return;
      }
      costmap_data_->data[y*costmap_data_->info.width+x] = value;
    }

    std::vector<std::vector<int8_t>> CostmapCore::getOccupancyGrid() const {
        return occupancy_grid_;
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::getCostmapData() const {
        return costmap_data_;
    }
}