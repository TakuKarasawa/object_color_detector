#ifndef MASK_IMAGE_CREATOR_H_
#define MASK_IMAGE_CREATOR_H_

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "color_param/color_param.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

class MaskImageCreator
{
public:
    MaskImageCreator();
    ~MaskImageCreator();
    void process();

private:
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    void load_color_param();
    void mask_image(cv::Mat& img);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber image_sub_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;

    XmlRpc::XmlRpcValue color_param_list_;
    std::vector<ColorParam> color_params_;

    std::string WINDOW_NAME_;
    int HZ_;
};

#endif  // MASK_IMAGE_CREATOR_H_
