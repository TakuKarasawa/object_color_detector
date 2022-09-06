cmake_minimum_required(VERSION 3.0.2)
project(object_color_detector_params)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    roscpp
    rospy
    std_msgs
)

generate_dynamic_reconfigure_options(
    cfg/Hsv.cfg
)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
