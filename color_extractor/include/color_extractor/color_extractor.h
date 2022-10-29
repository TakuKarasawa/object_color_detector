#ifndef COLOR_EXTRACTOR_H_
#define COLOR_EXTRACTOR_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ColorExtractor
{
public:
    ColorExtractor();
    ~ColorExtractor();
    void process();

private:
    void img_callback(const sensor_msgs::ImageConstPtr& msg);
    void extract(cv::Mat& src,cv::Mat& dst);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber img_sub_;

    bool is_bin_;
    int hue_min_;
    int hue_max_;
    int sat_min_;
    int sat_max_;
    int val_min_;
    int val_max_;

};

#endif  // COLOR_EXTRACTOR_H_
