#ifndef MASK_IMAGE_CREATOR_H_
#define MASK_IMAGE_CREATOR_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

// utility
#include "color_params/color_params.h"
#include "color_saver/color_saver.h"

namespace object_color_detector
{
enum Mode
{
    VIWER = 0,
    EXTRACTOR = 1
};

class MaskImageCreator
{
public:
    MaskImageCreator(Mode _mode,std::string _targte_color);
    ~MaskImageCreator();
    void process();

private:
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    void mask_image(cv::Mat& img);

    // node handle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber image_sub_;

    // image_transport
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;

    // color_params
    ColorParams* color_params_ptr_;

    // color_saver
    ColorSaver* color_saver_ptr_;

    // buffer
    Mode mode_;
    std::string target_color_;

    // parameters
    std::string WINDOW_NAME_;
    std::string FILE_PATH_;
    bool IS_SAVE_;
    HSV lower_;
    HSV upper_;
};
}

#endif  // MASK_IMAGE_CREATOR_H_