#include <chrono>
#include <memory>

#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  RCLCPP_INFO(this->get_logger(), "Hello, Console!");
  geometry_msgs::msg::Pose origin;
  origin.position.x = -20.0;
  origin.position.y = -20.0;
  origin.position.z = 0.0;
  origin.orientation.w = 1.0;

  costmap_.initializeCostmap(0.1, 100, 100, origin, 1);

  // string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const {
  
  costmap_.updateCostmap(msg);
  publishCostmap();
  
}

void CostmapNode::publishCostmap() const {
  
  auto costmap_msg = *costmap_.getCostmapData();
  costmap_msg.header.frame_id = "map";
  costmap_msg.header.stamp = this->now();

int width = costmap_msg.info.width;
int height = costmap_msg.info.height;
// RCLCPP_INFO(this->get_logger(), "Costmap width: %d, height: %d", width, height);
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
        int index = i * width + j;  // row-major order
        if (costmap_msg.data[index] > 0) {
            RCLCPP_INFO(this->get_logger(), "index: %d val: %d", index, costmap_msg.data[index]);
        }
    }
}
  costmap_pub_->publish(costmap_msg);
}

void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}