cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_object_color_detector)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    darknet_ros_msgs
    dynamic_reconfigure
    geometry_msgs
    image_transport
    object_color_detector_params
    object_detector_msgs
    object_color_detector_msgs
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
)

# OpenCV
find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 3.0 QUIET)
    if(NOT OpenCV_FOUND)
        find_package(OpenCV 3.4.3 QUIET)
        if(NOT OpenCV_FOUND)
            message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
        endif()
    endif()
endif()

# PCL
find_package(PCL REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

# point_cloud_object_color_detector_node
add_executable(point_cloud_object_color_detector_node
    src/point_cloud_object_color_detector.cpp
    src/point_cloud_object_color_detector_node.cpp
)
target_link_libraries(point_cloud_object_color_detector_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

# point_cloud_object_detector
add_executable(point_cloud_object_detector_node
    src/point_cloud_object_detector.cpp
    src/point_cloud_object_detector_node.cpp
)
target_link_libraries(point_cloud_object_detector_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
