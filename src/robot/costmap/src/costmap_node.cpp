#include <chrono>
#include <memory>

#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  costmap_.initializeCostmap(10.0, 10.0, 0.1);
  RCLCPP_INFO(this->get_logger(), "Hello, Console!");
  // string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  costmap_.initializeCostmap(10.0, 10.0, 0.1);
  RCLCPP_INFO(this->get_logger(), "Processing LaserScan message");
  int x_grid = 0, y_grid = 0;
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    double range = msg->ranges[i];
    double angle = msg->angle_min + i * msg->angle_increment;
    if (range > msg->range_max || range < msg->range_min) continue;

    double x = range * std::cos(angle);
    double y = range * std::sin(angle);
    x_grid = 5 + static_cast<int>(x/costmap_.getResolution());
    y_grid = 5 + static_cast<int>(y/costmap_.getResolution());

    costmap_.markObstacle(x_grid, y_grid);
    RCLCPP_INFO(this->get_logger(), "Obstacle at grid (%d, %d)", x_grid, y_grid);
  }
   RCLCPP_INFO(this->get_logger(), "FINISHED processing LaserScan message");
  costmap_.inflateObstacle(x_grid, y_grid, 1.0, 100.0);
  publishCostmap();
}

void CostmapNode::publishCostmap() {
  RCLCPP_INFO(this->get_logger(), "Publishing costmap data");
  
  auto grid_msg = nav_msgs::msg::OccupancyGrid();
  grid_msg.header.stamp = this->now();
  grid_msg.header.frame_id = "robot/chassis/lidar";
  
  grid_msg.info.resolution = costmap_.getResolution();
  grid_msg.info.width = costmap_.getWidth();
  grid_msg.info.height = costmap_.getHeight();

  grid_msg.info.origin.position.x = -(grid_msg.info.width * grid_msg.info.resolution) / 2.0;
  grid_msg.info.origin.position.y = -(grid_msg.info.height * grid_msg.info.resolution) / 2.0;

  grid_msg.data.resize(grid_msg.info.width * grid_msg.info.height, 0);
  auto occupancy_grid = costmap_.getOccupancyGrid();
  for (size_t i = 0; i < occupancy_grid.size(); ++i) {
    for (size_t j = 0; j < occupancy_grid[i].size(); ++j) {
      grid_msg.data[i * grid_msg.info.width + j] = occupancy_grid[i][j];
    }
  }
  costmap_pub_->publish(grid_msg);
  RCLCPP_INFO(this->get_logger(), "Costmap published");
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