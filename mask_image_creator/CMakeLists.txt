cmake_minimum_required(VERSION 3.0.2)
project(mask_image_creator)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    dynamic_reconfigure
    image_transport
    object_color_detector_params
    roscpp
    rospy
    sensor_msgs
    std_msgs
    color_dynamic_reconfigure
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

catkin_package(
    INCLUDE_DIRS include
    ${OpenCV_INCLUDE_DIRS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# mask_image_creator
add_executable(mask_image_creator_node
    src/mask_image_creator.cpp
    src/mask_image_creator_node.cpp
)
target_link_libraries(mask_image_creator_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
