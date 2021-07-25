#include <chrono>
#include <fstream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "ControllerNode.hpp"
#include "sensor_msgs/msg/camera_info.hpp"


using namespace std::chrono_literals;
namespace fs = std::filesystem;


ControllerNode::ControllerNode() : Node("controller_node")
{
    twist_publisher_ = this->create_publisher<Twist>("/surv_bot/cmd_vel", rclcpp::QoS(10));
    // RCLCPP_INFO(this->get_logger, "Publishing to /surv_bot/cmd_vel");
    laser_subscriber_ = this->create_subscription<LaserScan>("/laser1/scan", rclcpp::QoS(10), std::bind(&ControllerNode::laser_sub_callback, this, std::placeholders::_1));
    image_subscriber_ = this->create_subscription<Image>("/camera1/image_raw", rclcpp::QoS(10), std::bind(&ControllerNode::cam_sub_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(25ms, std::bind(&ControllerNode::publisher_callback, this));
    laser_scan_ = LaserScan();
    // twist_cmd_ = Twist();
}

void ControllerNode::publisher_callback()
{
    // Twist cmd = compute_twist_cmd();
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // RCLCPP_INFO(this->get_logger(), "Sending twist: ", cmd.linear.x);
    // std::cout << "sending twist: " << cmd.linear.x << std::endl;
    twist_publisher_->publish(compute_twist_cmd());

}

void ControllerNode::laser_sub_callback(LaserScan::SharedPtr msg)
{
    laser_scan_ = *msg.get();
    // RCLCPP_INFO(this->get_logger(), "Received laser");
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void ControllerNode::cam_sub_callback(Image::SharedPtr msg)
{
    cam_img_ = *msg.get();
    // RCLCPP_INFO(this->get_logger(), "Received camera image");
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

Twist ControllerNode::compute_twist_cmd()
{
    // for (float point : laser_scan_.ranges)
    // {
    //     std::cout << point << std::endl;
    // }
    // std::cout << "----------------" << std::endl;
    Twist twist_cmd = Twist();
    if (std::any_of(laser_scan_.ranges.begin(), laser_scan_.ranges.end(), [](float range)
                    { return range < 3.5; }))
    {
        RCLCPP_INFO(this->get_logger(), "Found obstacle");
        twist_cmd.linear.x = 0.0;

        save_img(cam_img_, fs::current_path().string());

        twist_cmd.angular.z = 0.5;
    }
    else
    {
        twist_cmd.linear.x = 1.5;
        twist_cmd.angular.z = 0.0;
        // for(float point : laser_scan_->ranges) std::cout << point << " ";
        // std::cout << std::endl;
        // RCLCPP_INFO(this->get_logger(), std::stof(laser_scan_->ranges));
    }
    return twist_cmd;
}

void ControllerNode::save_img(const Image &img, const std::string &images_folder)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        const std::string filename = images_folder + "/" + std::to_string(img.header.stamp.nanosec) + ".jpg";
        // char base_name[256];   //Image save
        // sprintf(base_name,"/home/zhy/catkin_ws_sliding_lane/image/%ld.jpg",count_++);
        cv::imwrite(filename, cv_ptr->image);
        std::cout << "Saved image of the obstacle" << std::endl;
    }
    catch(...)
    {
        std::cout << "Error saving image" << std::endl;
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}