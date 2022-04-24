#include "Fractals.h"
#include "../util/generators/Transformations.h"

void generateFractal(Figure& fig, Figures3D& fractal, const int nr_iterations, const double scale) {
    Figure base = fig;
    Matrix scaleMatrix = transformations::scaleFigure(1 / scale);

    // Go through all iterations
    for (int i = 0; i < nr_iterations; i++) {
        for (Figure figure : fractal) {
            for (Vector3D point : figure.points) {
                Vector3D newPoints;
                for (Vector3D point : figure.points) {}
                for (Face face : figure.faces) {}
                Figure scaled = Figure();
            }
        }
    }
}