#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
  ControlNode();

private:
  // Subscribers’ callbacks
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void poseCallback(const nav_msgs::msg::Odometry::ConstPtr &msg);
  //  ^ You might rename poseCallback to odomCallback (and adjust .cpp) for clarity.

  // Timer callback
  void controlLoop();

  // Internal helper methods
  int  findNearestWaypoint();
  float findDistance(float x1, float y1);
  float findDistanceIndexBased(int idx);
  int  idxCloseToLookahead(int idx);

  // Core control class (example from your snippet)
  robot::ControlCore control_;

  // Parameters
  int   lookahead_distance_;
  float max_linear_vel_;
  float max_angular_vel_;
  int   idx;

  // ROS communication
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr       path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr   odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    cmd_pub_;
  rclcpp::TimerBase::SharedPtr                               timer_;

  // Stored path and current waypoint index
  nav_msgs::msg::Path path_;
  size_t              current_wp_{0};

  // Robot state
  float xc{0.0f};
  float yc{0.0f};
  float yaw{0.0f};
  float vel{0.0f};

  // Example: wheelbase or track width (ensure it matches usage in .cpp)
  float WB{0.1f};

  // Example waypoint container: 2D array of (x, y)
  std::vector<std::vector<float>> waypoints;
};

#endif