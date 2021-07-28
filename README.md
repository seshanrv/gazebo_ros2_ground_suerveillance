# CPPND: Ground Surveillance robot using ROS2 and Gazebo

This is a starter repo for the Capstone project in the [Udacity C++ Nanodegree Program](https://www.udacity.com/course/c-plus-plus-nanodegree--nd213).

The Capstone Project gives you a chance to integrate what you've learned throughout this program. This project will become an important part of your portfolio to share with current and future colleagues and employers.

In this project, you can build your own C++ application starting with this repo, following the principles you have learned throughout this Nanodegree Program. This project will demonstrate that you can independently create applications using a wide range of C++ features.

Tested in Ubuntu 20.04

## Dependencies for Running Locally
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.2 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 9.3.0
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* ROS2 foxy
  * Linux (Ubuntu 20.04): [click here for installation instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html). Please follow the instructions to test ROS2 provided in the same page. It is convenient to append ```~/.bashrc``` with ```. <ros2-foxy installation path>/ros2-linux/setup.bash``` so that ROS2 is sourced everytime you open a terminal. 
* Gazebo11
  * Linux (Ubuntu 20.04): easiest way to install is from apt packages (assuming ros2 foxy is already installed)

  ```bash 
      sudo apt update \
      sudo apt install ros-foxy-gazebo-ros-pkgs
  ```
  . Click [here](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros) for more installation instructions. Please test Gazebo - ROS2 integration using the example in the link. 
* OpenCV >= 4.2
  * Linux (>= Ubuntu 18.04): click [here] (https://docs.opencv.org/4.5.0/d7/d9f/tutorial_linux_install.html) for installation instructions

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./HelloWorld`.