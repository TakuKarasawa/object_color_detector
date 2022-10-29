cmake_minimum_required(VERSION 3.0.2)
project(color_extractor)

add_compile_options(-o2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
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

catkin_package(
    INCLUDE_DIRS include
    ${OpenCV_INCLUDE_DIRS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# color_extractor
add_executable(color_extractor_node
    src/color_extractor.cpp
    src/color_extractor_node.cpp
)
target_link_libraries(color_extractor_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

