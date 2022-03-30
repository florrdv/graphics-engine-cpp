#include "Figure.h"

void Figure::triangulate() {
    std::vector<Face> newFaces = {};
    
    for (auto face : faces) {
        std::vector<Face> replacementFaces = face.triangulate();
        newFaces.insert(newFaces.end(), replacementFaces.begin(), replacementFaces.end());
    }

    faces = newFaces;
}