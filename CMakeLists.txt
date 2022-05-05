cmake_minimum_required(VERSION 3.0.2)
project(object_color_detector)

find_package(catkin REQUIRED COMPONENTS
  darknet_ros_msgs
  geometry_msgs
  object_detector_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
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
    LIBRARIES ${PROJECT_NAME}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

# object_color_detector_node
add_executable(object_color_detector_node
    src/object_color_detector.cpp
    src/object_color_detector_node.cpp
)
target_link_libraries(object_color_detector_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# mask_image_creator
add_executable(mask_image_creator_node
    src/mask_image_creator.cpp
    src/mask_image_creator_node.cpp
)
target_link_libraries(mask_image_creator_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})