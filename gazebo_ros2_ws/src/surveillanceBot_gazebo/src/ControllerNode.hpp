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

enum side{left, center, right};

class ControllerNode : public rclcpp::Node
{
    public:
        ControllerNode();
    private:
        void publisher_callback();
        void laser_sub_callback(LaserScan::SharedPtr msg);
        void cam_sub_callback(Image::SharedPtr msg);
        void save_img(const Image &img);
        Twist align_camera();
        Twist compute_twist_cmd();
        bool found_obstacle();
        
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<Twist>::SharedPtr twist_publisher_;
        rclcpp::Subscription<LaserScan>::SharedPtr laser_subscriber_;
        rclcpp::Subscription<Image>::SharedPtr image_subscriber_;
        
        LaserScan laser_scan_;
        Image cam_img_;
        bool camera_alignment_;
        bool img_saved_;
        side obstacle_side_;
};