#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <memory>
#include <vector>
#include <cmath>
#include <algorithm> 

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <cmath>
#include <chrono>
#include <memory>
#include <functional>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control")
  , control_(robot::ControlCore(this->get_logger()))
  , lookahead_distance_(0.5)
  , max_linear_vel_(0.5)
  , max_angular_vel_(0.5) {
	path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&ControlNode::poseCallback, this, std::placeholders::_1));
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!msg->poses.empty()) {
      path_ = *msg;
      size_t i = 0;
      waypoints.clear();
      for (auto &pose : path_.poses) {
        waypoints.push_back({(float)pose.pose.position.x, (float)pose.pose.position.y});
        i++;
      }
      current_wp_ = 0;
    }
}

void ControlNode::poseCallback(const nav_msgs::msg::Odometry::ConstPtr& msg)
{
    // Position
    xc = msg->pose.pose.position.x;
    yc = msg->pose.pose.position.y;
    float zc = msg->pose.pose.position.z;

    // Orientation
    float qx = msg->pose.pose.orientation.x;
    float qy = msg->pose.pose.orientation.y;
    float qz = msg->pose.pose.orientation.z;
    float qw = msg->pose.pose.orientation.w;

    // Linear velocity
    float linear_vx = msg->twist.twist.linear.x;
    float linear_vy = msg->twist.twist.linear.y;
    float linear_vz = msg->twist.twist.linear.z;

    std::vector<float> vect_vel = {linear_vx, linear_vy, linear_vz};
    float linear_vel = sqrt(linear_vx*linear_vx + linear_vy*linear_vy);

    // Angular velocity
    float angular_vel = msg->twist.twist.angular.z;

    // Euler from Quaternion
    tf2::Quaternion quat(qx, qy, qz, qw);
    tf2::Matrix3x3 mat(quat);

    double curr_roll, curr_pitch, curr_yaw;
    mat.getEulerYPR(curr_yaw, curr_pitch, curr_roll);

    // Assign to global variables
    vel = linear_vel;
    yaw = curr_yaw;
}

int ControlNode::findNearestWaypoint()
{
  int nearest_idx = 0;
  float smallest_dist = 0.;
  float P = 2.;
  for (int i = 0; i < waypoints.size(); i++)
  {
      float wpx = waypoints[i][0];
      float wpy = waypoints[i][1];
      float idx_dist = pow(xc - wpx, P) + pow(yc - wpy, P);

      if (i == 0) {
          smallest_dist = idx_dist;
          nearest_idx = i;
      }
      if (idx_dist < smallest_dist ) {
          smallest_dist = idx_dist;
          nearest_idx = i;
      }
  }
  RCLCPP_INFO(this->get_logger(), "%i: %f, %f", nearest_idx, waypoints[nearest_idx][0], waypoints[nearest_idx][1]);
  return nearest_idx;
}

void ControlNode::controlLoop() {
  if (path_.poses.size() > 0) {
    // Initialized variables
    RCLCPP_INFO(this->get_logger(), "starting!");
    int nearest_idx = findNearestWaypoint();
    if (nearest_idx <= 1) {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0;
      cmd.angular.z = 0;
      RCLCPP_INFO(this->get_logger(), "Arrived!");

      // Message is published outside of the function using
      cmd_pub_->publish(cmd);
      return;
    }
    idx = idxCloseToLookahead(nearest_idx);
    float target_x = waypoints[idx][0];
    float target_y = waypoints[idx][1];

    RCLCPP_INFO(this->get_logger(), "got the nearest idx!");

    // calculate alpha (angle between the goal point and the path point)
    float x_delta = target_x - xc;
    float y_delta = target_y - yc;
    float alpha = atan2(y_delta, x_delta) - yaw;

    // front of the vehicle is 0 degrees right +90 and left -90 hence we need to convert our alpha
    while (alpha > M_PI) alpha -= 2.0 * M_PI;
    while (alpha < -M_PI) alpha += 2.0 * M_PI;

    // fet lookahead distance depending on the speed
    float lookahead = findDistance(target_x, target_y);
    float steering_angle = atan((2. * WB * alpha) / lookahead);

    RCLCPP_INFO(this->get_logger(), "got the lookahead distance and steering angle!");

    // set max wheel turning angle
    if (steering_angle > 0.7) {
    steering_angle = 0.7;
    } else if (steering_angle < -0.7) {
    steering_angle = -0.7;
    }

    // Publish the message
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = max_linear_vel_;
    cmd.angular.z = steering_angle;
    RCLCPP_INFO(this->get_logger(), "got the cmd!");

    // Message is published outside of the function using
    cmd_pub_->publish(cmd);
  }
}

float ControlNode::findDistance(float x1, float y1)
{
  float P = 2.0; // power of 2
  float distance = sqrt(pow(x1 - xc, P) + pow(y1 - yc, P));
  return distance;
}

float ControlNode::findDistanceIndexBased(int idx)
{
  float x1 = waypoints[idx][0];
  float y1 = waypoints[idx][1];
  return findDistance(x1, y1);
}

int ControlNode::idxCloseToLookahead(int idx)
{
  while (findDistanceIndexBased(idx) < lookahead_distance_) {
      idx += 1;
      if (idx == waypoints.size()) { break; }
  }
  return idx - 1;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}