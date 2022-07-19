cmake_minimum_required(VERSION 3.0.2)
project(color_dynamic_reconfigure)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    object_color_detector_params
    roscpp
    rospy
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
