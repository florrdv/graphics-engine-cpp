#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_POINT2D_H_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_POINT2D_H_

class Point2D {
public:
    double x;
    double y;

    Point2D(double a, double b): x(a), y(b) {};
    Point2D(): x(0.0), y(0.0) {};
};

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_POINT2D_H_