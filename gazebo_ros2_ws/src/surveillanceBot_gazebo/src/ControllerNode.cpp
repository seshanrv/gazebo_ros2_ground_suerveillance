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
    laser_subscriber_ = this->create_subscription<LaserScan>("/laser1/scan", rclcpp::QoS(10), std::bind(&ControllerNode::laser_sub_callback, this, std::placeholders::_1));
    image_subscriber_ = this->create_subscription<Image>("/camera1/image_raw", rclcpp::QoS(10), std::bind(&ControllerNode::cam_sub_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(25ms, std::bind(&ControllerNode::publisher_callback, this));
    laser_scan_ = LaserScan();
    found_obstacle_prev_ = false;
    found_obstacle_ = false;
    camera_alignment_ = false;
    img_saved_ = false;
    obstacle_side_ = center;
}

void ControllerNode::publisher_callback()
{
    twist_publisher_->publish(compute_twist_cmd());
}

void ControllerNode::laser_sub_callback(LaserScan::SharedPtr msg)
{
    laser_scan_ = *msg.get();
}

void ControllerNode::cam_sub_callback(Image::SharedPtr msg)
{
    cam_img_ = *msg.get();
}

bool ControllerNode::found_obstacle()
{
    bool found_obst = std::any_of(laser_scan_.ranges.begin(), laser_scan_.ranges.end(), [](float range)
                    { return range < 3.5; });

    return found_obst;
}

Twist ControllerNode::compute_twist_cmd()
{
    Twist twist_cmd = Twist();
    if (found_obstacle() && !camera_alignment_ && !img_saved_)
        twist_cmd = align_camera();

    else if (found_obstacle() && camera_alignment_ && !img_saved_)
        save_img(cam_img_);
    else if (found_obstacle() && img_saved_)
    {
        twist_cmd.linear.x = 0.0;
        twist_cmd.angular.z = obstacle_side_ == left ? 0.5 : -0.5;
    }
    else
    {
        twist_cmd.linear.x = 1.0;
        twist_cmd.angular.z = 0.0;
        img_saved_ = false;
        camera_alignment_ = false;
    }
    return twist_cmd;
}

void ControllerNode::save_img(const Image &img)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        const std::string image_folder = fs::current_path().string() + "/camera_images";

        if (!fs::is_directory(image_folder))
            fs::create_directory(image_folder);

        const std::string filename = image_folder + "/" + std::to_string(img.header.stamp.nanosec) + ".jpg";
        cv::imwrite(filename, cv_ptr->image);
        RCLCPP_INFO(this->get_logger(), "Saved image of the obstacle");
        img_saved_ = true;
    }
    catch (...)
    {
        RCLCPP_INFO(this->get_logger(), "Error saving image");
    }
}

Twist ControllerNode::align_camera()
{
    RCLCPP_INFO(this->get_logger(), "Aligning camera...");

    Twist twist_cmd;

    if (laser_scan_.ranges[laser_scan_.ranges.size() / 2] == INFINITY &&
        std::any_of(laser_scan_.ranges.begin() + laser_scan_.ranges.size() / 2,
                    laser_scan_.ranges.end(),
                    [](float range)
                    { return range != INFINITY; }))
    {
        obstacle_side_ = right;
        twist_cmd.linear.x = 0.0;
        twist_cmd.angular.z = 0.3;
    }
    else if (laser_scan_.ranges[laser_scan_.ranges.size() / 2] == INFINITY)
    {
        obstacle_side_ = left;
        twist_cmd.linear.x = 0.0;
        twist_cmd.angular.z = -0.3;
    }
    else
    {
        twist_cmd.linear.x = 0.0;
        twist_cmd.angular.z = 0.0;
        camera_alignment_ = true;
    }
    return twist_cmd;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}