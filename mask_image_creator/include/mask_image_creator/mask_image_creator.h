#ifndef MASK_IMAGE_CREATOR_H_
#define MASK_IMAGE_CREATOR_H_

#include <ros/ros.h>

class MaskImageCreator
{
public:
    MaskImageCreator();
    void process();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    int HZ_;
};

#endif  // MASK_IMAGE_CREATOR_H_
