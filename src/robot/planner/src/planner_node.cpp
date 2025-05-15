#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  
  map_updated_ = false;
  goal_updated_ = false;
  // Initialize subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
  "/map", 10,
  std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
  "/goal_pose", 10,
  std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
  "/odom/filtered", 10,
  std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Initialize publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Initialize timer (1 second interval)
  timer_ = this->create_wall_timer(
  std::chrono::milliseconds(500),
  std::bind(&PlannerNode::updatePath, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  global_map = *msg;
  map_updated_ = true;
  this->pathPlan();
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  goal_updated_ = true;
  current_goal = *msg;
  this->pathPlan();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pose = *msg;
}

void PlannerNode::pathPlan() {
  if(map_updated_ && goal_updated_) {
    robot::PlannerCore::CellIndex goal;
    goal.x = (current_goal.pose.position.x - global_map.info.origin.position.x) / global_map.info.resolution;
    goal.y = (current_goal.pose.position.y - global_map.info.origin.position.y) / global_map.info.resolution;

    robot::PlannerCore::CellIndex home;
    home.x = (current_pose.pose.pose.position.x - global_map.info.origin.position.x) / global_map.info.resolution;
    home.y = (current_pose.pose.pose.position.y - global_map.info.origin.position.y) / global_map.info.resolution;
    
    planner_.plan(global_map, goal, home);
  }
}

void PlannerNode::updatePath() {
  path_pub_->publish(planner_.path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}