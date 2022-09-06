#ifndef HSV_H_
#define HSV_H_

#include <iostream>

class HSV
{
public:
    HSV() :
        h(0), s(0), v(0) {}

    HSV(int _h,int _s,int _v) :
        h(_h), s(_s), v(_v) {}

    int h;
    int s;
    int v;

private:
};

#endif  // HSV_H_
