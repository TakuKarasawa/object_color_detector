#include "mask_image_creator/mask_image_creator.h"

MaskImageCreator::MaskImageCreator() :
    private_nh_("~")
{
    private_nh_.param("HZ",HZ_,{10});
}

void MaskImageCreator::process()
{
    ros::Rate rate(HZ_);
    while(ros::ok()){
        std::cout << "hello" << std::endl;
        ros::spinOnce();
        rate.sleep();
    }
}
