#include "Fractals.h"
#include "../util/generators/Transformations.h"

void generateFractal(Figure& fig, Figures3D& fractal, const int nr_iterations, const double scale) {
    Figure base = fig;
    Matrix scaleMatrix = transformations::scaleFigure(1 / scale);

    // Go through all iterations
    for (int i = 0; i < nr_iterations; i++) {
        for (Figure figure : fractal) {
            for (Vector3D point : figure.points) {
                std::vector<Vector3D> newPoints;
                std::vector<Face> newFaces;
                for (Vector3D point : figure.points) newPoints.push_back(point * scaleMatrix);
                for (Face face : figure.faces) newFaces.push_back(face.clone());
                Figure scaled = Figure(newPoints, newFaces, figure.color);
            }
        }
    }
}