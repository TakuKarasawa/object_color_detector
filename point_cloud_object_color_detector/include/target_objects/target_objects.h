#ifndef TARGET_OBJECTS_H_
#define TARGET_OBJECTS_H_

#include <ros/ros.h>

namespace object_color_detector
{
class TargetObjects : public std::vector<std::string>
{
public:
    TargetObjects() :
        private_nh_("~") { load_yaml(); }

    TargetObjects(ros::NodeHandle _private_nh) :
        private_nh_(_private_nh) { load_yaml(); }

    bool is_included(std::string name)
    {
        for(auto it = this->begin(); it != this->end(); it++) if(name == *it) return true;
        return false;
    }

    // for debug
    void print_elements()
    {
        for(auto it = this->begin(); it != this->end(); it++) std::cout << "name: " << *it << std::endl;
        std::cout << std::endl;
    }

private:
    void load_yaml()
    {
        std::string yaml_file_name;
        private_nh_.param("YAML_FILE_NAME",yaml_file_name,{std::string("target_objects")});
        XmlRpc::XmlRpcValue target_objects;
        if(!private_nh_.getParam(yaml_file_name.c_str(),target_objects)){
            ROS_ERROR("Cloud not load %s", yaml_file_name.c_str());
            return;
        }
        ROS_ASSERT(target_objects.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for(int i = 0; i < (int)target_objects.size(); i++){
            if(!target_objects[i]["name"].valid()){
                ROS_ERROR("%s is valid", yaml_file_name.c_str());
                return;
            }
            if(target_objects[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString){
                std::string name = static_cast<std::string>(target_objects[i]["name"]);
                this->emplace_back(name);
            }
        }
    }

    ros::NodeHandle private_nh_;
};
} // namespace object_color_detector

#endif	// TARGET_OBJECTS_H_