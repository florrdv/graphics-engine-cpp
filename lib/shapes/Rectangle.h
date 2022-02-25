#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_LIB_SHAPES_RECTANGLE_H_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_LIB_SHAPES_RECTANGLE_H_

#include "Shape.h"

class Rectangle: protected Shape {
public:
    Rectangle(int w, int h): Shape(w, h) {};
};

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_LIB_SHAPES_RECTANGLE_H_