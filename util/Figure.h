#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_FIGURE_H_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_FIGURE_H_

#include <vector>

#include "Face.h"
#include "Color.h"

#include "../lib/vector3d/vector3d.h"

class Figure {
public:
    std::vector<Vector3D> points;
    std::vector<Face> faces;
    Color color;
};

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_FIGURE_H_