#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_LINE2D_H_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_LINE2D_H_

#include "Point2D.h"
#include "Color.h"

class Line2D {
public:
    Point2D p1;
    Point2D p2;

    Color color;

    double z1;
    double z2;

    Line2D(Point2D p, Point2D q, double zO, double zT, Color c): p1(p), p2(q), color(c), z1(zO), z2(zT) {};
};

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_LINE2D_H_