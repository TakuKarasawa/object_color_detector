#include "mask_image_creator/mask_image_creator.h"

MaskImageCreator::MaskImageCreator() :
	private_nh_("~"), it_(nh_)
{
	private_nh_.param("HZ",HZ_,{10});
	if(!private_nh_.getParam("WINDOW_NAME",WINDOW_NAME_)) WINDOW_NAME_ = "Mask Image";

	image_sub_ = nh_.subscribe("/camera/color/image_rect_color",1,&MaskImageCreator::image_callback,this);
	image_pub_ = it_.advertise("/mask_image",1);

	load_color_param();
	cv::namedWindow(WINDOW_NAME_);
}

MaskImageCreator::~MaskImageCreator() { cv::destroyAllWindows(); }

void MaskImageCreator::load_color_param()
{
	if(!private_nh_.getParam("color_param",color_param_list_)){
		ROS_WARN("Cloud not load color_param_list");
		return;
	}
	ROS_ASSERT(color_param_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i = 0; i < (int)color_param_list_.size(); i++){
        if(!color_param_list_[i]["name"].valid() || 
		   !color_param_list_[i]["lower_h"].valid() || !color_param_list_[i]["lower_s"].valid() || !color_param_list_[i]["lower_v"].valid() || 
		   !color_param_list_[i]["upper_h"].valid() || !color_param_list_[i]["upper_s"].valid() || !color_param_list_[i]["upper_v"].valid()){
			   ROS_WARN("object_list is valid");
			   return;
        }
        if(color_param_list_[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString && 
		   color_param_list_[i]["lower_h"].getType() == XmlRpc::XmlRpcValue::TypeInt && color_param_list_[i]["lower_s"].getType() == XmlRpc::XmlRpcValue::TypeInt && color_param_list_[i]["lower_v"].getType() == XmlRpc::XmlRpcValue::TypeInt &&
		   color_param_list_[i]["upper_h"].getType() == XmlRpc::XmlRpcValue::TypeInt && color_param_list_[i]["upper_s"].getType() == XmlRpc::XmlRpcValue::TypeInt && color_param_list_[i]["upper_v"].getType() == XmlRpc::XmlRpcValue::TypeInt){
			   std::string name = static_cast<std::string>(color_param_list_[i]["name"]);
			   int lower_h = static_cast<int>(color_param_list_[i]["lower_h"]);
			   int lower_s = static_cast<int>(color_param_list_[i]["lower_s"]);
			   int lower_v = static_cast<int>(color_param_list_[i]["lower_v"]);
			   int upper_h = static_cast<int>(color_param_list_[i]["upper_h"]);
			   int upper_s = static_cast<int>(color_param_list_[i]["upper_s"]);
			   int upper_v = static_cast<int>(color_param_list_[i]["upper_v"]);

			   HSV lower(lower_h,lower_s,lower_v);
			   HSV upper(upper_h,upper_s,upper_v);
			   ColorParam color_param(name,lower,upper);
			   color_params_.push_back(color_param);
        }
    }
}

void MaskImageCreator::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
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

	cv::imshow(WINDOW_NAME_,output_img);
	cv::waitKey(1);

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