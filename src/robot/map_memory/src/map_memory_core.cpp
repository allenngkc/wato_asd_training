#include <eigen3/Eigen/Core>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
: logger_(logger),
  last_x(0.0),
  last_y(0.0),
  last_yaw(0.0),
  distance_threshold(2.0)
{
    // define the global map
    global_map_.info.resolution = 0.2;           
    global_map_.info.width = 30/global_map_.info.resolution;
    global_map_.info.height = 30/global_map_.info.resolution;
    // origin position = (0,0,0), orientation = identity
    global_map_.info.origin.position.x = -15;
    global_map_.info.origin.position.y = -15;
    global_map_.info.origin.position.z = 0.0;
    global_map_.info.origin.orientation.x = 0.0;
    global_map_.info.origin.orientation.y = 0.0;
    global_map_.info.origin.orientation.z = 0.0;
    global_map_.info.origin.orientation.w = 1.0;
    global_map_.header.frame_id = "sim_world";

    global_map_.data.resize(
        global_map_.info.width * global_map_.info.height, 
        -1  // unknown cells
    );
}

void MapMemoryCore::integrateCostmap()
{
    // RCLCPP_INFO(logger_, "yaw: %f", last_yaw);
    for (size_t i = 0; i < global_map_.data.size(); i++) {
        // convert the chosen cell's coordinates to local map
        int w = global_map_.info.width;
        double res = global_map_.info.resolution;
        double x = (int(i % w) + 0.5) * res;
        double y = (int(i / w) + 0.5) * res;

        // transform to local frame
        Eigen::Vector2d p_local;
        p_local << x + (global_map_.info.origin.position.x - last_x),
                   y + (global_map_.info.origin.position.y - last_y);
        Eigen::Matrix2d rot_mat;
        rot_mat << cos(last_yaw), sin(last_yaw),
                   -sin(last_yaw), cos(last_yaw);
                   
        p_local = rot_mat * p_local;  // transform to the lidar frame (rotated by yaw)
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
    // RCLCPP_INFO(logger_, "last pos: %f, %f", last_x, last_y);
    // RCLCPP_INFO(logger_, "latest map: %i", latest_costmap_.data.size());
}

} 