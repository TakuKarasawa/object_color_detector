#ifndef COLOR_PARAM_H_
#define COLOR_PARAM_H_

#include <iostream>
#include "hsv.h"

namespace object_color_detector
{
class ColorParam
{
public:
    ColorParam() :
        name(std::string("")), lower(HSV()), upper(HSV()) {}

    ColorParam(std::string _name,HSV _lower,HSV _upper) :
        name(_name), lower(_lower), upper(_upper) {}

    std::string name;
    HSV lower;
    HSV upper;

private:
};
}

#endif  // COLOR_PARAM_H_