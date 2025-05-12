#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);
    
    void initMapMemory(double resolution, int width, int height, const geometry_msgs::msg::Pose origin);
    void integrateCostmap();

    void setCostmapUpdated(bool val);
    void setShouldUpdateMap(bool val);
    void setlatestCostmap(nav_msgs::msg::OccupancyGrid msg);
    void setlatestOdom(nav_msgs::msg::Odometry msg);
    void setLastX(double x);
    void setLastY(double y);

    double getLastX();
    double getLastY();
    double getDistanceThreshold();
    bool getCostmapUpdate();
    bool getShouldUpdateMap();
    nav_msgs::msg::OccupancyGrid getGlobalMap();

    double last_yaw = 0.0;

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid global_map_;

    // global_map_.header.stamp = rclcpp::Time(0);
    // global_map_.header.frame_id = "sim_world";
    // global_map_.info.resolution = 0.1; 
    // global_map_.info.width = 100;       
    // global_map_.info.height = 100;    
    // global_map_.info.origin.position.x = -20.0;
    // global_map_.info.origin.position.y = -20.0;
    // global_map_.info.origin.position.z = 0.0;
    // global_map_.info.origin.orientation.w = 1.0;
    // global_map_.data.resize(global_map_.info.width * global_map_.info.height, -1);
    
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    nav_msgs::msg::Odometry latest_odom_;
    double last_x, last_y;
    double distance_threshold = 1.5;
    bool costmap_updated_ = false;
    bool should_update_map_ = false;
};

}  

#endif  
