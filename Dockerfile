FROM ros:foxy

# install ros package
RUN apt-get update && apt-get install -y \
	ros-foxy-gazebo-ros-pkgs \
	x11vnc \
	xvfb \
	openbox
	
RUN rm -rf /var/lib/apt/lists/*
RUN mkdir ~/.vnc

# windowing for terminal and gazebo window
RUN echo "openbox &" >> /.bashrc
#autostart gazebo and ros node
RUN echo "cd /home/gazebo_ros2_project && gazebo arena_1.sdf & " >> /.bashrc
RUN echo "cd /home/gazebo_ros2_project/gazebo_ros2_ws && colcon build" >> /.bashrc
RUN echo "source install/setup.bash" >> /.bashrc
RUN echo "ros2 run surveillance_bot_gazebo ControllerNode">> /.bashrc
