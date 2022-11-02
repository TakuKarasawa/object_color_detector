# object_color_detector

## Environment
- Ubuntu 20.04
- ROS Noetic
- PCL 1.10
- [darknet_ros](https://github.com/leggedrobotics/darknet_ros)
- [object_detector](https://github.com/TakuKarasawa/object_detector)

## point_cloud_object_color_detector
### Topics
#### Subscribed topics
- /bounding_boxes (darknet_ros_msgs/BoundingBoxes)
- /camera/depth_registered/points (sensor_msgs/PointCloud2)

#### Published topics
- /object_positions ([object_detector_msgs/ObjectPositions](https://github.com/TakuKarasawa/object_detector/blob/master/object_detector_msgs/msg/ObjectPositions.msg))
- /object_color_positions ([object_color_detector_msgs::ObjectColorPositions](https://github.com/TakuKarasawa/object_color_detector/tree/master/object_color_detector_msgs))

### How to Use
- start darknet_ros(ex. when you use yolov3)
```
roslaunch darknet_ros yolo_v3.launch
```

- start point_cloud_object_color_detector_node
```
roslaunch point_cloud_object_color_detector point_cloud_object_detector.launch
```

## mask_image_creator
### Topics
#### Subscribed topics
- /camera/color/image_rect_color (sensor_msgs::Image)

### How to Use
- start mask_image_creator
```
roslaunch mask_image_creator mask_image_creator.launch
```