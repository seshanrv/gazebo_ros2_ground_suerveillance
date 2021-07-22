#include <cstdio>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher_base.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

void subscriber_callback(sensor_msgs::msg::Image::SharedPtr msg){
  std::cout << "Received image: height = " << msg->height << "width" << msg->width << std::endl;
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world surveillanceBot_gazebo package\n");

  rclcpp::init(1, argv);

  auto node = rclcpp::Node::make_shared("bot_cntrl");

  auto veclocity_pub = node->create_publisher<geometry_msgs::msg::Twist>("/surv_bot/cmd_vel", rclcpp::QoS(10));
  auto img_sub = node->create_subscription<sensor_msgs::msg::Image>("/camera1/image_raw", rclcpp::QoS(10), &subscriber_callback);

  auto  velocity_cmd = geometry_msgs::msg::Twist();

  velocity_cmd.linear.x = 1.0;



  for(int i =0; i<10; i++){
    veclocity_pub->publish(velocity_cmd); 
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
