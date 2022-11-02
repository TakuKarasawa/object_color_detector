#ifndef COLOR_SAVER_H_
#define COLOR_SAVER_H_

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>

#include "color_param/color_param.h"

namespace object_color_detector
{
class ColorSaver
{
public:
    ColorSaver() {}

    void save_color_params(std::string file_path,std::vector<ColorParam>& color_params)
    {   
        std::string file_name = file_path + get_date() + ".yaml";
        static std::ofstream ofs(file_name);
        std::cout << "Save: " + file_name << std::endl;
        ofs << "color_params:" << std::endl;
        for(const auto &cp : color_params){
            ofs << "  - name: " << cp.name << std::endl;
            ofs << "    lower_h: " << cp.lower.h << std::endl;
            ofs << "    lower_s: " << cp.lower.s << std::endl;
            ofs << "    lower_v: " << cp.lower.v << std::endl;
            ofs << "    upper_h: " << cp.upper.h << std::endl;
            ofs << "    upper_s: " << cp.upper.s << std::endl;
            ofs << "    upper_v: " << cp.upper.v << "\n" << std::endl;
        }
        ofs.close();
    }

private:
    std::string get_date()
    {
        time_t t = time(nullptr);
        const tm* localTime = localtime(&t);
        std::stringstream s;
        s << localTime->tm_year + 1900;
        s << std::setw(2) << std::setfill('0') << localTime->tm_mon + 1;
        s << std::setw(2) << std::setfill('0') << localTime->tm_mday;
        s << std::setw(2) << std::setfill('0') << localTime->tm_hour;
        s << std::setw(2) << std::setfill('0') << localTime->tm_min;
        s << std::setw(2) << std::setfill('0') << localTime->tm_sec;

        return s.str();
    }  
};
}

#endif  // COLOR_SAVER_H_