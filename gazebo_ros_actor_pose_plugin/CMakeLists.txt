cmake_minimum_required(VERSION 3.0.2)
project(gazebo_ros_actor_pose_plugin)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include/${PROJECT_NAME} ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
)

add_library(gazebo_ros_actor_pose SHARED src/gazebo_ros_actor_pose.cpp)
target_link_libraries(gazebo_ros_actor_pose ${caktin_LIBRARIES} ${GAZEBO_LIBRARIES})
