#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}
 
    void CostmapCore::initializeCostmap(double width, double height, double resolution) {
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    occupancy_grid_.clear();  
    occupancy_grid_.resize(height_);
    for (auto &row : occupancy_grid_) {
        std::fill(row.begin(), row.end(), 0);
    }
    }

    void CostmapCore::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);
        
    x_grid = static_cast<int>((x - (-(width_ * resolution_) / 2.0)) / resolution_);
    y_grid = static_cast<int>((y - (-(height_ * resolution_) / 2.0)) / resolution_);
    }

    void CostmapCore::inflateObstacle(int x, int y, double radius, double max_cost) {
    int inflation_cells = static_cast<int>(radius / resolution_);
    for (int i = -inflation_cells; i <= inflation_cells; ++i) {
        for (int j = -inflation_cells; j <= inflation_cells; ++j) {
            int nx = x + i;
            int ny = y + j;

            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                double distance = std::sqrt(i*i+j*j) * resolution_;
                if (distance > inflation_cells) continue;

                double cost = max_cost * (1.0 - distance / inflation_cells);
                setOccupancyGrid(nx, ny, std::max(occupancy_grid_[nx][ny], static_cast<int8_t>(cost)));
            }
        }
    } 
    }

    void CostmapCore::markObstacle(int x, int y) {
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            occupancy_grid_[y][x] = 100;
        }
    }

    std::vector<std::vector<int8_t>> CostmapCore::getOccupancyGrid() const {
        return occupancy_grid_;
    }

    double CostmapCore::getWidth() const {
        return width_;
    }

    double CostmapCore::getHeight() const {
        return height_;
    }

    double CostmapCore::getResolution() const {
        return resolution_;
    }
}