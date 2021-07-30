FROM ros:foxy

# install ros package
RUN apt-get update && apt-get install -y \
#      ros-${ROS_DISTRO}-demo-nodes-cpp \
#      ros-${ROS_DISTRO}-demo-nodes-py && \
	ros-foxy-gazebo-ros-pkgs \
	x11vnc \
	xvfb 
	
RUN rm -rf /var/lib/apt/lists/*
RUN mkdir ~/.vnc

# Setup a password
#RUN     x11vnc -storepasswd 1234 ~/.vnc/passwd
# Autostart gazebo
RUN bash -c 	'echo "cd /home/gazebo_ros2_project && gazebo arena_1.sdf" >> /.bashrc'
#RUN bash -c 	'cd /home/gazebo_ros2_project/gazebo_ros2_ws && \
#		colcon build '
#RUN bash -c	'. install/setup.bash && \
#		ros2 run surveillance_bot_gazebo ControllerNode'


# launch ros package
#CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener.launch.py"]
#CMD gazebo
