# Use the official ROS Noetic image as the base image
FROM osrf/ros:noetic-desktop-full

# Install dependencies
RUN apt-get update && apt-get install -y \
    wget \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install PyBullet using pip
RUN pip3 install pybullet

# Upgrade numpy
RUN pip3 install --upgrade numpy

# Install catkin_tools
RUN pip3 install catkin_tools

# Set up catkin workspace
RUN mkdir -p /root/catkin_ws/src 

# Copy your local ROS packages into the container's catkin workspace
COPY . /root/catkin_ws/src/

# Build the workspace with local packages using catkin build
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin build"

# Set the default shell to bash
CMD ["bash"]
