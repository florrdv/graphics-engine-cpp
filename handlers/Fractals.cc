#include "Fractals.h"
#include "ZBufferTriangle.h"
#include "../util/generators/Transformations.h"
#include "../util/generators/PlatonicSolids.h"
#include <cmath>

void generateFractal(Figure& fig, Figures3D& fractal, const int nr_iterations, const double scale) {
    if (nr_iterations == 0) return;

    fractal.push_back(fig);
    Matrix scaleMatrix = transformations::scaleFigure(1 / scale);

    // Go through all iterations
    Figures3D intermediate;
    for (Figure figure : fractal) {
        for (Vector3D point : figure.points) {
            Figure scaled = figure;
            Vector3D translation = point - point * scaleMatrix;

            std::vector<Vector3D> newPoints;
            for (Vector3D point : figure.points) newPoints.push_back(point * scaleMatrix + translation);

            scaled.points = newPoints;
            intermediate.push_back(scaled);
        }
    }

    fractal = intermediate;

    return generateFractal(fig, fractal, nr_iterations - 1, scale);
}

std::pair<double, double> getDimensions(Figures3D &figures) {
    
}

void generateMengerSponge(Figures3D &figures, const int iteration, const int nr_iterations) {
    if (iteration == nr_iterations) { return; }
    Matrix scaleMatrix = transformations::scaleFigure(1.0 / 9.0);
    // figures.push_back(base);

    

    Figures3D newFigures;
    for (int x = 0; x < 3; x++) {
        for (int y = 0; y < 3; y++) {
            for (int z = 0; z < 3; z++) {
                int i = 0;
                if (x == 1) i++;
                if (y == 1) i++;
                if (z == 1) i++;
                if (i >= 2) continue;
                auto scale = iteration * 4;
                Vector3D translation = Vector3D::point(x * (2 + scale), y * (2 + scale), z * (2 + scale));
                for (Figure &f : figures) {
                    Figure b = f;
                    for (Vector3D &point : b.points) {
                        point += translation;
                    }
                    newFigures.push_back(b);
                }
            }
        }
    }

    figures = newFigures;

    return generateMengerSponge(figures, iteration + 1, nr_iterations);
}