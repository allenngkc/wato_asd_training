#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "map_memory_node.hpp"
#include "map_memory_core.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
    map_update_done_ = true;
    costmap_updated_ = false;
    should_update_map_ = false;
    // Initialize subscribers
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

    // Initialize publisher
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    // Initialize timer
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MapMemoryNode::publishMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    costmap_updated_ = true;
    // Store the latest costmap
    if (map_update_done_ && should_update_map_) {
        map_memory_.latest_costmap_ = *msg;
        this->updateMap();
    }
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    
    double r_x = msg->pose.pose.orientation.x;
    double r_y = msg->pose.pose.orientation.y;
    double r_z = msg->pose.pose.orientation.z;
    double r_w = msg->pose.pose.orientation.w;

    double yaw = std::atan2(2.0 * (r_w * r_z + r_x * r_y), 1.0 - 2.0 * (r_y * r_y + r_z * r_z));  

    // Compute distance traveled
    double distance = std::sqrt(std::pow(x - map_memory_.last_x, 2) + std::pow(y - map_memory_.last_y, 2));
    if (distance >= map_memory_.distance_threshold) {
        map_memory_.last_x = x;
        map_memory_.last_y = y;
        map_memory_.last_yaw = yaw;
        should_update_map_ = true;
    }
}

void MapMemoryNode::updateMap()
{
    if (costmap_updated_) {
        map_update_done_ = false;
        map_memory_.integrateCostmap();
        should_update_map_ = false;
        map_update_done_ = true;
    }
}

void MapMemoryNode::publishMap() {
    map_memory_.global_map_.header.stamp = this->now();
    map_memory_.global_map_.header.frame_id = "sim_world";
    map_pub_->publish(map_memory_.global_map_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}