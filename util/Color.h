#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_MISC_UTIL_COLOR_CC_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_MISC_UTIL_COLOR_CC_

#include "../easy_image.h"

class Color {
public:
    double red;
    double green;
    double blue;

    Color(double r, double g, double b): red(r), green(g), blue(b) {};
    img::Color toNative();

    Color& operator+=(const Color& rhs) {
        red += rhs.red;
        green += rhs.green;
        blue += rhs.blue;

        return *this;
    }

    Color& operator*=(const Color& rhs) {
        red *= rhs.red;
        green *= rhs.green;
        blue *= rhs.blue;

        return *this;
    }

    friend Color operator+(Color& lhs, const Color& rhs){
        return Color(lhs.red + rhs.red, lhs.green + rhs.green, lhs.blue + rhs.blue);
    }

    friend Color operator*(Color& lhs, const Color& rhs){
        return Color(lhs.red * rhs.red, lhs.green * rhs.green, lhs.blue * rhs.blue);
    }
};

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_MISC_UTIL_COLOR_CC_