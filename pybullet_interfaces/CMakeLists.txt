cmake_minimum_required(VERSION 3.0.2)
project(pybullet_interfaces)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)
add_service_files(DIRECTORY srv FILES EndEffectorControl.srv)
generate_messages(DEPENDENCIES geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)