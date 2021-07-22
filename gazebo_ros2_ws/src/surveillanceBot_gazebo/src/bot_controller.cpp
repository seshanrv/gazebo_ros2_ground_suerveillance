#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher_base.hpp"
#include "geometry_msgs/msg/twist.hpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world surveillanceBot_gazebo package\n");

  rclcpp::init(1, argv);

  auto node = rclcpp::Node::make_shared("bot_cntrl");

  auto veclocity_pub = node->create_publisher<geometry_msgs::msg::Twist>("/surv_bot/cmd_vel", rclcpp::QoS(10));

  auto  velocity_cmd = geometry_msgs::msg::Twist();

  velocity_cmd.linear.x = 1.0;

  for(int i =0; i<10; i++){
    veclocity_pub->publish(velocity_cmd); 
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();

  return 0;
}
