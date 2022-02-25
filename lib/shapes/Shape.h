#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_LIB_SHAPES_SHAPE_H_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_LIB_SHAPES_SHAPE_H_

class Shape {
protected:
    int width;
    int height;

public:
    Shape(int w, int h): width(h), height(h) {};
};

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_LIB_SHAPES_SHAPE_H_