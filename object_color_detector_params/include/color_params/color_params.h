#ifndef COLOR_PARAMS_H_
#define COLOR_PARAMS_H_

#include <ros/ros.h>
#include "color_param/color_param.h"

namespace object_color_detector
{
class ColorParams : public std::vector<ColorParam>
{
public:
    ColorParams() :
        private_nh_("~") { load_yaml(); }

    ColorParams(ros::NodeHandle _private_nh) :
        private_nh_(_private_nh) { load_yaml(); }

    void output_color_params(std::vector<ColorParam>& color_params)
    {
        color_params.clear();
        for(auto it = this->begin(); it != this->end(); it++){
            ColorParam color_param(it->name,it->lower,it->upper);
            color_params.emplace_back(color_param);
        }
    }

    void output_color_param(ColorParam& color_param,std::string color)
    {
        for(auto it = this->begin(); it != this->end(); it++){
            if(it->name == color){
                color_param.name = color;
                color_param.lower = it->lower;
                color_param.upper = it->upper;
            }
        }
    }    

    HSV get_lower(std::string color)
    {
        for(auto it = this->begin(); it != this->end(); it++){
            if(it->name == color) return it->lower;
        }
        return HSV();
    }

    HSV get_upper(std::string color)
    {
        for(auto it = this->begin(); it != this->end(); it++){
            if(it->name == color) return it->upper;
        }
        return HSV();
    }

    // for debug
    void print_elements()
    {
        for(auto it = this->begin(); it != this->end(); it++){
            std::cout << "Name: " << it->name << std::endl;
            std::cout << "lower(" << it->lower.h << ","
                                  << it->lower.s << ","
                                  << it->lower.v << ")" << std::endl;
            std::cout << "upper(" << it->upper.h << ","
                                  << it->upper.s << ","
                                  << it->upper.v << ")" << std::endl;
        }
        std::cout << std::endl;
    }

private:
    void load_yaml()
    {
        std::string yaml_file_name;
        private_nh_.param("YAML_FILE_NAME",yaml_file_name,{std::string("color_params")});
        XmlRpc::XmlRpcValue color_params;
        if(!private_nh_.getParam(yaml_file_name.c_str(),color_params)){
            ROS_WARN("Cloud not load %s", yaml_file_name.c_str());
            return;
        }

        ROS_ASSERT(color_params.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for(int i = 0; i < (int)color_params.size(); i++){
            if(!color_params[i]["name"].valid() ||
               !color_params[i]["lower_h"].valid() || !color_params[i]["lower_s"].valid() || !color_params[i]["lower_v"].valid() ||
               !color_params[i]["upper_h"].valid() || !color_params[i]["upper_s"].valid() || !color_params[i]["upper_v"].valid()){
                ROS_WARN("%s is valid", yaml_file_name.c_str());
                return;
            }
            if(color_params[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString &&
               color_params[i]["lower_h"].getType() == XmlRpc::XmlRpcValue::TypeInt &&
               color_params[i]["lower_s"].getType() == XmlRpc::XmlRpcValue::TypeInt &&
               color_params[i]["lower_v"].getType() == XmlRpc::XmlRpcValue::TypeInt &&
               color_params[i]["upper_h"].getType() == XmlRpc::XmlRpcValue::TypeInt &&
               color_params[i]["upper_s"].getType() == XmlRpc::XmlRpcValue::TypeInt &&
               color_params[i]["upper_v"].getType() == XmlRpc::XmlRpcValue::TypeInt){
                std::string name = static_cast<std::string>(color_params[i]["name"]);
                int lower_h = static_cast<int>(color_params[i]["lower_h"]);
                int lower_s = static_cast<int>(color_params[i]["lower_s"]);
                int lower_v = static_cast<int>(color_params[i]["lower_v"]);
                int upper_h = static_cast<int>(color_params[i]["upper_h"]);
                int upper_s = static_cast<int>(color_params[i]["upper_s"]);
                int upper_v = static_cast<int>(color_params[i]["upper_v"]);

                HSV lower(lower_h,lower_s,lower_v);
                HSV upper(upper_h,upper_s,upper_v);
                ColorParam color_param(name,lower,upper);
                this->emplace_back(color_param);
            }
        }
    }

    ros::NodeHandle private_nh_;
};
}

#endif  // COLOR_PARAMS_H_