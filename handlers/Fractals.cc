#include "Fractals.h"
#include "../util/generators/Transformations.h"

void generateFractal(Figure& fig, Figures3D& fractal, const int nr_iterations, const double scale) {
    Figure base = fig;

    // Go through all iterations
    for (int i = 0; i < nr_iterations; i++) {
        for (auto f : fractal) {
            for (auto p : f.points) {
                Matrix scaleMatrix = transformations::scaleFigure(1 / scale);
                Matrix
            }
        }
    }
}