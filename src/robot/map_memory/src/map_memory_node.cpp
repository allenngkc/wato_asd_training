#include "map_memory_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
  timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

  geometry_msgs::msg::Pose origin;
  origin.position.x = -20.0;
  origin.position.y = -20.0;
  origin.position.z = 0.0;
  origin.orientation.w = -1.0;
  map_memory_.initMapMemory(0.1, 100, 100, origin);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_memory_.setlatestCostmap(*msg);
  map_memory_.setCostmapUpdated(true);
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  map_memory_.setlatestOdom(*msg);
  double distance = std::sqrt(std::pow(x - map_memory_.getLastX(), 2) + std::pow(y - map_memory_.getLastY(), 2));
  if (distance >= map_memory_.getDistanceThreshold()) {
    map_memory_.setLastX(x);
    map_memory_.setLastY(y);
    map_memory_.setShouldUpdateMap(true);
  }
}

void MapMemoryNode::updateMap() {
  if (map_memory_.getCostmapUpdate() && map_memory_.getShouldUpdateMap()) {
    map_memory_.integrateCostmap();
    map_pub_->publish(map_memory_.getGlobalMap());
    map_memory_.setCostmapUpdated(false);

  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
