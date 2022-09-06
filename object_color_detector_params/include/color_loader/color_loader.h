#ifndef COLOR_LOADER_H_
#define COLOR_LOADER_H_

#include <ros/ros.h>
#include "color_param/color_param.h"

class ColorLoader
{
public:
    ColorLoader() :
        private_nh_("~") {}
    ColorLoader(ros::NodeHandle _nh,ros::NodeHandle _private_nh) :
        nh_(_nh), private_nh_(_private_nh) {}

    void output_color_params(std::vector<ColorParam>& color_params)
    {
        color_params_.clear();
        load_color_params();
        color_params = color_params_;
    }


private:
    void load_color_params()
    {
        private_nh_.param("COLOR_PARAMS_NAME",COLOR_PARAMS_NAME_,{std::string("color_params")});
        XmlRpc::XmlRpcValue color_param_list;
        if(!private_nh_.getParam(COLOR_PARAMS_NAME_.c_str(),color_param_list)){
            ROS_WARN("Cloud not load color_param_list");
            return;
        }

        ROS_ASSERT(color_param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for(int i = 0; i < (int)color_param_list.size(); i++){
            if(!color_param_list[i]["name"].valid() ||
               !color_param_list[i]["lower_h"].valid() || !color_param_list[i]["lower_s"].valid() || !color_param_list[i]["lower_v"].valid() ||
               !color_param_list[i]["upper_h"].valid() || !color_param_list[i]["upper_s"].valid() || !color_param_list[i]["upper_v"].valid()){
                ROS_WARN("color_param_list is valid");
                return;
            }
            if(color_param_list[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString &&
               color_param_list[i]["lower_h"].getType() == XmlRpc::XmlRpcValue::TypeInt &&
               color_param_list[i]["lower_s"].getType() == XmlRpc::XmlRpcValue::TypeInt &&
               color_param_list[i]["lower_v"].getType() == XmlRpc::XmlRpcValue::TypeInt &&
               color_param_list[i]["upper_h"].getType() == XmlRpc::XmlRpcValue::TypeInt &&
               color_param_list[i]["upper_s"].getType() == XmlRpc::XmlRpcValue::TypeInt &&
               color_param_list[i]["upper_v"].getType() == XmlRpc::XmlRpcValue::TypeInt){
                std::string name = static_cast<std::string>(color_param_list[i]["name"]);
                int lower_h = static_cast<int>(color_param_list[i]["lower_h"]);
                int lower_s = static_cast<int>(color_param_list[i]["lower_s"]);
                int lower_v = static_cast<int>(color_param_list[i]["lower_v"]);
                int upper_h = static_cast<int>(color_param_list[i]["upper_h"]);
                int upper_s = static_cast<int>(color_param_list[i]["upper_s"]);
                int upper_v = static_cast<int>(color_param_list[i]["upper_v"]);

                HSV lower(lower_h,lower_s,lower_v);
                HSV upper(upper_h,upper_s,upper_v);
                ColorParam color_param(name,lower,upper);
                color_params_.push_back(color_param);
            }
        }
    }

    // node handle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // color_params
    std::string COLOR_PARAMS_NAME_;
    std::vector<ColorParam> color_params_;
};

#endif  // COLOR_LOADER_H_
