#include <cstdio>
// #include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher_base.hpp"
#include "geometry_msgs/msg/twist.hpp"
// #include "gazebo_plugins/gazebo_ros_diff_drive.hpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world surveillanceBot_gazebo package\n");

  // const std::string cmd_vel_topic = "/surv_bot/cmd_vel";
  rclcpp::init(1, argv);

  auto node = rclcpp::Node::make_shared("bot_cntrl");

  auto veclocity_pub = node->create_publisher<geometry_msgs::msg::Twist>("/surv_bot/cmd_vel", rclcpp::QoS(10));

  auto  velocity_cmd = geometry_msgs::msg::Twist();

  velocity_cmd.linear.x = 1.0;

  veclocity_pub->publish(velocity_cmd); 

  rclcpp::shutdown();

  return 0;
}
