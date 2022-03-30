#include "Face.h"

std::vector<Face> Face::triangulate() {
    int n = pointIndexes.size();
    std::vector<Face> newFaces = {};
    
    for (int i = 1; i <= n-2; n++) { 
        std::vector<int> points = {};
        points.push_back(0);
        points.push_back(i);
        points.push_back(i+1);
        
        Face newFace = Face(points);
        newFaces.push_back(newFace);
    }

    return newFaces;
}