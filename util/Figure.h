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
    Color ambientReflection = Color(0, 0, 0);
    Color diffuseReflection = Color(0, 0, 0);
    Color specularReflection = Color(0, 0, 0);
    double reflectionCoefficient = 0;

    Figure(std::vector<Vector3D> p, std::vector<Face> f, Color a, Color d) : points(p), faces(f), ambientReflection(a), diffuseReflection(d) {};
    Figure() {};
    void triangulate();
};

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_FIGURE_H_
