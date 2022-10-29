#include "color_extractor/color_extractor.h"

ColorExtractor::ColorExtractor() :
	private_nh_("~"), is_bin_(false),
	hue_min_(0), hue_max_(180),
	sat_min_(0), sat_max_(255),
	val_min_(0), val_max_(255)
{
	img_sub_ = nh_.subscribe("img_in",1,&ColorExtractor::img_callback,this);
	// img_sub_ = nh_.subscribe("roomba2/camera/color/image_rect_color",1,&ColorExtractor::img_callback,this);

	cv::namedWindow("Image");
	cv::createTrackbar("HueMin","Image",&hue_min_,180);
	cv::createTrackbar("HueMax","Image",&hue_max_,180);
	cv::createTrackbar("SatMin","Image",&sat_min_,255);
	cv::createTrackbar("SatMax","Image",&sat_max_,255);
	cv::createTrackbar("ValMin","Image",&val_min_,255);
	cv::createTrackbar("ValMax","Image",&val_max_,255);
}

ColorExtractor::~ColorExtractor() { cv::destroyAllWindows(); }

void ColorExtractor::img_callback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& ex){
        ROS_ERROR("Could not convert to color image");
        return;
    }

	if(cv_ptr->image.empty()){
		ROS_WARN("Image is empty");
		return;
	}

	cv::Mat dst;
	extract(cv_ptr->image,dst);
	cv::imshow("Image",dst);
	if(cv::waitKey(10) == 'b') is_bin_ = !is_bin_;
}

void ColorExtractor::extract(cv::Mat& src,cv::Mat& dst)
{
	cv::Mat hsv;
	cv::cvtColor(src,hsv,CV_BGR2HSV);
	dst = src.clone();
	for(int y = 0; y < hsv.rows; y++){
		for(int x = 0; x < hsv.cols; x++){
			cv::Vec3b data = hsv.at<cv::Vec3d>(y,x);
			if(data[0] < hue_min_ || data[0] > hue_max_ ||
			   data[1] < sat_min_ || data[1] > sat_max_ ||
			   data[2] < val_min_ || data[2] > val_max_){
				dst.at<cv::Vec3b>(y,x) = cv::Vec3b(0,0,0);
			}
			else if(is_bin_){
				dst.at<cv::Vec3b>(y,x) = cv::Vec3b(255,255,255);
			}
		}
	}
}

void ColorExtractor::process() { ros::spin(); }