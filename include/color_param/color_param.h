#ifndef COLOR_PARAM_H_
#define COLOR_PARAM_H_

#include <iostream>

struct HSV
{
	HSV() :
		h(0), s(0), v(0) {}

	HSV(int _h,int _s,int _v) :
		h(_h), s(_s), v(_v) {}

	int h;
	int s;
	int v;
};

struct ColorParam
{
	ColorParam(std::string _name,HSV _lower,HSV _upper) :
		name(_name), lower(_lower), upper(_upper) {}

	std::string name;
	HSV lower;
	HSV upper;
};

#endif	// COLOR_PARAM_H_