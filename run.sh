#!/bin/bash

docker run --rm -p 5900:5900 \
	-e HOME=/ \
	-v "$PWD":/home/gazebo_ros2_project:rw \
	gazebo_ros2:0.0 \
	x11vnc -forever -create 

