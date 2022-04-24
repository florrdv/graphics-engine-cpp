#include "Face.h"

std::vector<Face> Face::triangulate() {
    int n = pointIndexes.size();
    std::vector<Face> newFaces = {};
    
    for (int i = 1; i <= n-2; i++) {
        std::vector<int> points = {};
        points.push_back(pointIndexes[0]);
        points.push_back(pointIndexes[i]);
        points.push_back(pointIndexes[i+1]);
        
        Face newFace = Face(points);
        newFaces.push_back(newFace);
    }

    return newFaces;
}

Face Face::clone() const {
    std::vector<int> newPointIndexes(pointIndexes);

    return Face(newPointIndexes);
};