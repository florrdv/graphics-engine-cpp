#include "Fractals.h"
#include "ZBufferTriangle.h"
#include "../util/generators/Transformations.h"
#include "../util/generators/PlatonicSolids.h"
#include <cmath>

void generateFractal(Figure& fig, Figures3D& fractal, const int nr_iterations, const double scale) {
    if (fractal.empty()) fractal.push_back(fig);
    if (nr_iterations == 0) return;

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

Vector3D getDimensions(Figures3D &figures) {
     std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;

    for (Figure &f : figures) {
        for (Vector3D &p : f.points) {
            x.push_back(p.x);
            y.push_back(p.y);
            z.push_back(p.z);
        }
    }

    Vector3D min = Vector3D::point(
        *std::min_element(x.begin(), x.end()), 
        *std::min_element(y.begin(), y.end()), 
        *std::min_element(z.begin(), z.end())
    );

    Vector3D max = Vector3D::point(
        *std::max_element(x.begin(), x.end()), 
        *std::max_element(y.begin(), y.end()), 
        *std::max_element(z.begin(), z.end())
    );  

   return max - min;
}

void generateMengerSponge(Figures3D &figures, const int iteration, const int nr_iterations) {
    if (iteration == nr_iterations) { return; }
    Matrix scaleMatrix = transformations::scaleFigure(1.0 / 3);

    Vector3D dimensions = getDimensions(figures);

    Figures3D newFigures;
    for (int x = -1; x < 2; x++) {
        for (int y = -1; y < 2; y++) {
            for (int z = -1; z < 2; z++) {
                int i = 0;
                if (x == 0) i++;
                if (y == 0) i++;
                if (z == 0) i++;
                if (i >= 2) continue;

                Vector3D translation = Vector3D::point(x * dimensions.x, y * dimensions.x, z * dimensions.x);
                for (Figure &f : figures) {
                    Figure b = f;
                    for (Vector3D &point : b.points) {
                        point += translation;
                        point *= scaleMatrix;
                    }
                    newFigures.push_back(b);
                }
            }
        }
    }

    figures = newFigures;

    return generateMengerSponge(figures, iteration + 1, nr_iterations);
}