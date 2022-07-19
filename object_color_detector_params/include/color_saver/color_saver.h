#ifndef COLOR_SAVER_H_
#define COLOR_SAVER_H_

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>

#include "color_param/color_param.h"

class ColorSaver
{
public:
    ColorSaver() {}

    void save_color_params(std::string file_name,std::vector<ColorParam>& color_params)
    {
        static std::ofstream ofs(file_name);
        for(const auto &cp : color_params){
            ofs << cp.name << "[";
            ofs << std::fixed << std::setprecision(14)
                << cp.lower.h << "," << cp.lower.s << "," << cp.lower.v << ","
                << cp.upper.h << "," << cp.upper.s << "," << cp.upper.v << "]\n";
        }
    }

private:
};

#endif  // COLOR_SAVER_H_
