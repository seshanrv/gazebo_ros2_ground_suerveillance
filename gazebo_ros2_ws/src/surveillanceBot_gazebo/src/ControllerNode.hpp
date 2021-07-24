#include <cstdio>
#include <iostream>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher_base.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using geometry_msgs::msg::Twist;
using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::Image;

class ControllerNode : public rclcpp::Node
{
    public:
        ControllerNode();
    private:
        void publisher_callback();
        void laser_sub_callback(LaserScan::SharedPtr msg);
        void cam_sub_callback(Image::SharedPtr msg);
        
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<Twist>::SharedPtr twist_publisher_;
        rclcpp::Subscription<LaserScan>::SharedPtr laser_subscriber_;
        rclcpp::Subscription<Image>::SharedPtr image_subscriber_;
        
        void compute_twist_cmd();
        LaserScan* laser_scan_;
        Image* cam_img_;
        Twist twist_cmd_;
};