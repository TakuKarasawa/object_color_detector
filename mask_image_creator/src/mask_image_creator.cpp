#include "mask_image_creator/mask_image_creator.h"

using namespace object_color_detector;

MaskImageCreator::MaskImageCreator(Mode _mode,std::string _target_color) :
    private_nh_("~"), 
    color_params_ptr_(new ColorParams(private_nh_)),
    mode_(_mode), 
    target_color_(_target_color)
{
    private_nh_.param("FILE_PATH",FILE_PATH_,{std::string("")});

    private_nh_.param("IS_SAVE",IS_SAVE_,{false});
    if(IS_SAVE_) color_saver_ptr_ = new ColorSaver();

    if(!private_nh_.getParam("WINDOW_NAME",WINDOW_NAME_)) WINDOW_NAME_ = std::string("Mask Image");
    cv::namedWindow(WINDOW_NAME_);
    if(mode_ == Mode::EXTRACTOR){
        cv::createTrackbar("HueMin",WINDOW_NAME_,&lower_.h,180);
	    cv::createTrackbar("HueMax",WINDOW_NAME_,&upper_.h,180);
	    cv::createTrackbar("SatMin",WINDOW_NAME_,&lower_.s,255);
	    cv::createTrackbar("SatMax",WINDOW_NAME_,&upper_.s,255);
	    cv::createTrackbar("ValMin",WINDOW_NAME_,&lower_.v,255);
	    cv::createTrackbar("ValMax",WINDOW_NAME_,&upper_.v,255);
    }

    if(mode_ == Mode::VIWER){
        lower_ = color_params_ptr_->get_lower(target_color_);
        upper_ = color_params_ptr_->get_upper(target_color_);
    }

    image_sub_ = nh_.subscribe("/img_in",1,&MaskImageCreator::image_callback,this);
    image_pub_ = nh_.advertise<sensor_msgs::Image>("img_out",1);  
}

MaskImageCreator::~MaskImageCreator() 
{ 
    cv::destroyAllWindows();
    if(mode_ == Mode::EXTRACTOR){
        if(IS_SAVE_){
            std::vector<ColorParam> color_params;
            for(auto it = color_params_ptr_->begin(); it != color_params_ptr_->end(); it++){
                if(it->name != target_color_) color_params.emplace_back(*it);          
                else{
                    ColorParam color_param;
                    color_param.name = target_color_;
                    color_param.lower = lower_;
                    color_param.upper = upper_;
                    color_params.emplace_back(color_param);
                }
            }

            if(FILE_PATH_ == std::string("")) return;
            color_saver_ptr_->save_color_params(FILE_PATH_,color_params);
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
    cv::Scalar scalar_min = cv::Scalar(lower_.h,lower_.s,lower_.v);
    cv::Scalar scalar_max = cv::Scalar(upper_.h,upper_.s,upper_.v);
    cv::inRange(hsv_img,scalar_min,scalar_max,mask_img);
    img.copyTo(output_img,mask_img);

    cv::Mat base(cv::max(img.rows,output_img.rows),img.cols + output_img.cols,CV_8UC3);
    cv::Mat roi_1(base,cv::Rect(0,0,img.cols,img.rows));
    img.copyTo(roi_1);
    cv::Mat roi_2(base,cv::Rect(img.cols,0,output_img.cols,output_img.rows) );
    output_img.copyTo(roi_2);

    cv::imshow(WINDOW_NAME_,base);
    cv::waitKey(10);

    
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",output_img).toImageMsg();
    image_pub_.publish(img_msg);
}

void MaskImageCreator::process() { ros::spin(); }