#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_MISC_UTIL_COLOR_CC_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_MISC_UTIL_COLOR_CC_

#include <algorithm>
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
        red = std::min(red, 1.00);
        green += rhs.green;
        green = std::min(green, 1.00);
        blue += rhs.blue;
        blue = std::min(blue, 1.00);

        return *this;
    }

    Color& operator*=(const Color& rhs) {
        red *= rhs.red;
        red = std::min(red, 1.00);
        green *= rhs.green;
        green = std::min(green, 1.00);
        blue *= rhs.blue;
        blue = std::min(blue, 1.00);

        return *this;
    }

    friend Color operator*(Color& lhs, const double& alpha) {
        return Color(std::min(lhs.red * alpha, 1.00), std::min(lhs.green * alpha, 1.00), std::min(lhs.blue * alpha, 1.00));
    }

    friend Color operator+(Color& lhs, const Color& rhs){
        return Color(std::min(lhs.red + rhs.red, 1.00), std::min(lhs.green + rhs.green, 1.00), std::min(lhs.blue + rhs.blue, 1.00));
    }

    friend Color operator*(Color& lhs, const Color& rhs){
        return Color(std::min(lhs.red * rhs.red, 1.00), std::min(lhs.green * rhs.green, 1.00), std::min(lhs.blue * rhs.blue, 1.00));
    }
};

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_MISC_UTIL_COLOR_CC_