#include <chrono>
#include "ControllerNode.hpp"

using namespace std::chrono_literals;


ControllerNode::ControllerNode() : Node("controller_node")
{
    twist_publisher_ = this->create_publisher<Twist>("/surv_bot/cmd_vel", rclcpp::QoS(10));
    // RCLCPP_INFO(this->get_logger, "Publishing to /surv_bot/cmd_vel");
    laser_subscriber_ = this->create_subscription<LaserScan>("/laser1/scan", rclcpp::QoS(10), std::bind(&ControllerNode::laser_sub_callback, this, std::placeholders::_1));
    image_subscriber_ = this->create_subscription<Image>("/camera1/image_raw", rclcpp::QoS(10), std::bind(&ControllerNode::cam_sub_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(500ms, std::bind(&ControllerNode::publisher_callback, this));
    twist_cmd_ = Twist();
}

void ControllerNode::publisher_callback()
{
    compute_twist_cmd();
    twist_publisher_->publish(ControllerNode::twist_cmd_);

}

void ControllerNode::laser_sub_callback(LaserScan::SharedPtr msg)
{
    laser_scan_ = msg.get();
    std::cout << "Received laser: stamp = " << msg->header.stamp.nanosec << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void ControllerNode::cam_sub_callback(Image::SharedPtr msg)
{
    cam_img_ = msg.get();
    std::cout << "Received image: height = " << msg->height << "width" << msg->width << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void ControllerNode::compute_twist_cmd()
{   
    float range_max = laser_scan_->range_max;
    if (std::any_of(laser_scan_->ranges.begin(), laser_scan_->ranges.end(), [range_max](float range){return range < range_max;}))
        twist_cmd_.linear.x = 0.0;
    else
        twist_cmd_.linear.x =1.0;
}
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}