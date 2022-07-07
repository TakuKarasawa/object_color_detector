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

// Custom msg
#include "color_param/color_param.h"
#include "color_loader/color_loader.h"
#include "color_saver/color_saver.h"
#include "color_dynamic_reconfigure/color_dynamic_reconfigure.h"

class MaskImageCreator
{
public:
    MaskImageCreator();
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

    // color_reconfigure
    std::shared_ptr<ColorDynamicReconfigure> color_dr_ptr_;

    // color_loader
    std::shared_ptr<ColorLoader> color_loader_ptr_;
    std::vector<ColorParam> color_params_;

    // color_saver
    std::shared_ptr<ColorSaver> color_saver_ptr_;

    // parameters
    std::string WINDOW_NAME_;
    std::string FILE_NAME_;
    bool IS_WINDOW_;
    bool IS_DR_;
    bool IS_SAVE_;
    int HZ_;
};

#endif  // MASK_IMAGE_CREATOR_H_
