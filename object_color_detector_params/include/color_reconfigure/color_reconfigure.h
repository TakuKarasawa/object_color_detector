#ifndef COLOR_RECONFIGURE_H_
#define COLOR_RECONFIGURE_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "color_param/color_param.h"
#include "object_color_detector_param/HsvConfig.h"

class ColorReconfigure
{
public:
	ColorReconfigure() :
		private_nh_("~") { init(); }

	ColorReconfigure(ros::NodeHandle _private_nh) :
		private_nh_(_private_nh) { init(); }

private:
	void init()
	{
		color_server_ = std::make_shared<dynamic_reconfigure::Server<object_color_detector_param::HsvConfig>>(private_nh_);
		color_callback = boost::bind(&ColorReconfigure::callback,this,_1,_2);
		color_server_->setCallback(color_callback);
	}

	void callback(object_color_detector_param::HsvConfig& config,uint32_t level)
	{
		std::cout << "LOWER_RED_H: " << config.LOWER_RED_H << std::endl;
	}

	// node handle
	ros::NodeHandle private_nh_;

	// dynamic reconfigure
	std::shared_ptr<dynamic_reconfigure::Server<object_color_detector_param::HsvConfig>> color_server_;
	dynamic_reconfigure::Server<object_color_detector_param::HsvConfig>::CallbackType color_callback;
};

#endif  // COLOR_RECONFIGURE_H_