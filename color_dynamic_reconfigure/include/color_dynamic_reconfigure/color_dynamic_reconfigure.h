#ifndef COLOR_DYNAMIC_RECONFIGURE_H_
#define COLOR_DYNAMIC_RECONFIGURE_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "color_param/color_param.h"
#include "object_color_detector_params/HsvConfig.h"

class ColorDynamicReconfigure
{
public:
    ColorDynamicReconfigure() :
        private_nh_("~") { init(); }
    ColorDynamicReconfigure(ros::NodeHandle _private_nh) :
        private_nh_(_private_nh) { init(); }

    void input_color_params(std::vector<ColorParam> color_param) { color_params_ = color_param; }
    void output_color_params(std::vector<ColorParam>& color_params) { color_params = color_params_; }

private:
    void init()
    {
        color_server_ = std::make_shared<dynamic_reconfigure::Server<object_color_detector_params::HsvConfig>>(private_nh_);
		color_callback = boost::bind(&ColorDynamicReconfigure::callback,this,_1,_2);
		color_server_->setCallback(color_callback);
    }

    void callback(object_color_detector_params::HsvConfig& config,uint32_t level)
	{
        for(auto &c : color_params_){
            if(c.name == "RED"){
                c.lower.h = config.LOWER_RED_H;
                c.lower.s = config.LOWER_RED_S;
                c.lower.v = config.LOWER_RED_V;
                c.upper.h = config.UPPER_RED_H;
                c.upper.s = config.UPPER_RED_S;
                c.upper.v = config.UPPER_RED_V;
            }
            else if(c.name == "BLUE"){
                c.lower.h = config.LOWER_BLUE_H;
                c.lower.s = config.LOWER_BLUE_S;
                c.lower.v = config.LOWER_BLUE_V;
                c.upper.h = config.UPPER_BLUE_H;
                c.upper.s = config.UPPER_BLUE_S;
                c.upper.v = config.UPPER_BLUE_V;
            }
            else if(c.name == "YELLOW"){
                c.lower.h = config.LOWER_YELLOW_H;
                c.lower.s = config.LOWER_YELLOW_S;
                c.lower.v = config.LOWER_YELLOW_V;
                c.upper.h = config.UPPER_YELLOW_H;
                c.upper.s = config.UPPER_YELLOW_S;
                c.upper.v = config.UPPER_YELLOW_V;
            }
            else if(c.name == "GREEN"){
                c.lower.h = config.LOWER_GREEN_H;
                c.lower.s = config.LOWER_GREEN_S;
                c.lower.v = config.LOWER_GREEN_V;
                c.upper.h = config.UPPER_GREEN_H;
                c.upper.s = config.UPPER_GREEN_S;
                c.upper.v = config.UPPER_GREEN_V;
            }
            else if(c.name == "ORANGE"){
                c.lower.h = config.LOWER_ORANGE_H;
                c.lower.s = config.LOWER_ORANGE_S;
                c.lower.v = config.LOWER_ORANGE_V;
                c.upper.h = config.UPPER_ORANGE_H;
                c.upper.s = config.UPPER_ORANGE_S;
                c.upper.v = config.UPPER_ORANGE_V;
            }
            else if(c.name == "PURPLE"){
                c.lower.h = config.LOWER_PURPLE_H;
                c.lower.s = config.LOWER_PURPLE_S;
                c.lower.v = config.LOWER_PURPLE_V;
                c.upper.h = config.UPPER_PURPLE_H;
                c.upper.s = config.UPPER_PURPLE_S;
                c.upper.v = config.UPPER_PURPLE_V;
            }
            else{
                std::cout << "No applicable color" << std::endl;
            }
        }
	}

    // node handle
    ros::NodeHandle private_nh_;

    // dynamic reconfigure
	std::shared_ptr<dynamic_reconfigure::Server<object_color_detector_params::HsvConfig>> color_server_;
	dynamic_reconfigure::Server<object_color_detector_params::HsvConfig>::CallbackType color_callback;

    // color param
    std::vector<ColorParam> color_params_;
};

#endif  // COLOR_DYNAMIC_RECONFIGURE_H_