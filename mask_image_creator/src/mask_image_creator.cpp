#include "mask_image_creator/mask_image_creator.h"

MaskImageCreator::MaskImageCreator() :
    private_nh_("~"), it_(nh_)
{
    if(!private_nh_.getParam("WINDOW_NAME",WINDOW_NAME_)) WINDOW_NAME_ = std::string("Mask Image");
    private_nh_.param("FILE_NAME",FILE_NAME_,{std::string("")});
    private_nh_.param("IS_WINDOW",IS_WINDOW_,{false});
    private_nh_.param("IS_DR",IS_DR_,{false});
    private_nh_.param("IS_SAVE",IS_SAVE_,{false});
    private_nh_.param("HZ",HZ_,{10});

    // color_loader
    color_loader_ptr_ = std::make_shared<ColorLoader>(nh_,private_nh_);
    color_loader_ptr_->output_color_params(color_params_);

    // color_reconfigure
    if(IS_DR_){
        color_dr_ptr_ = std::make_shared<ColorDynamicReconfigure>(private_nh_);
        color_dr_ptr_->input_color_params(color_params_);    
    }

    if(IS_SAVE_) color_saver_ptr_ = std::make_shared<ColorSaver>();

    image_sub_ = nh_.subscribe("/camera/color/image_rect_color",1,&MaskImageCreator::image_callback,this);
    image_pub_ = it_.advertise("/mask_image",1);

    cv::namedWindow(WINDOW_NAME_);
}

MaskImageCreator::~MaskImageCreator() 
{ 
    cv::destroyAllWindows(); 
    color_saver_ptr_->save_color_params(FILE_NAME_,color_params_);
}

void MaskImageCreator::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    if(IS_DR_) color_dr_ptr_->output_color_params(color_params_);

    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        mask_image(cv_ptr->image);
    }
    catch(cv_bridge::Exception& ex){
        ROS_ERROR("Could not convert to color image");
        return;
    }
}

void MaskImageCreator::mask_image(cv::Mat& img)
{
    cv::Mat hsv_img, mask_img, output_img;
    cv::cvtColor(img,hsv_img,CV_BGR2HSV,3);
    cv::Scalar s_min = cv::Scalar(color_params_[3].lower.h,
                                  color_params_[3].lower.s,
                                  color_params_[3].lower.v);
    cv::Scalar s_max = cv::Scalar(color_params_[3].upper.h,
                                  color_params_[3].upper.s,
                                  color_params_[3].upper.v);
    cv::inRange(hsv_img,s_min,s_max,mask_img);

    img.copyTo(output_img,mask_img);

    if(IS_WINDOW_){
        cv::imshow(WINDOW_NAME_,output_img);
        cv::waitKey(1);
    }

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",output_img).toImageMsg();
    image_pub_.publish(img_msg);
}

void MaskImageCreator::process()
{
    ros::Rate rate(HZ_);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}
