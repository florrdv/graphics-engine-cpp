#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_MISC_UTIL_COLOR_CC_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_MISC_UTIL_COLOR_CC_

#include "../easy_image.h";

class Color {
public:
    double red;
    double green;
    double blue;

    Color(double r, double g, double b): red(r), green(r), blue(b) {};
    img::Color toNative();
};

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_MISC_UTIL_COLOR_CC_