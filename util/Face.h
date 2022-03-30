#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_FACE_H_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_FACE_H_

#include <vector>

class Face { 
public: 
    std::vector<int> pointIndexes;

    Face(std::vector<int> p) : pointIndexes(p) {};
    std::vector<Face> triangulate();
};

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_FACE_H_