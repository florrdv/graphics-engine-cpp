#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_LINE2D_H_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_LINE2D_H_

#include "Point2D.h"
#include "Color.h"

class Line2D {
public:
    Point2D p1;
    Point2D p2;

    Color color;

    Line2D(Point2D p, Point2D q, Color c): p1(p), p2(q), color(c) {};
};

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_LINE2D_H_