#include "Fractals.h"
#include "ZBufferTriangle.h"
#include "../util/generators/Transformations.h"
#include "../util/generators/PlatonicSolids.h"
#include <cmath>

void generateFractal(Figure& fig, Figures3D& fractal, const int nr_iterations, const double scale) {
    fractal.push_back(fig);
    Matrix scaleMatrix = transformations::scaleFigure(1 / scale);

    // Go through all iterations
    for (int i = 0; i < nr_iterations; i++) {
        Figures3D intermediate;
        for (Figure figure : fractal) {
            for (Vector3D point : figure.points) {
                Vector3D scaledPoint = point * scaleMatrix;
                Vector3D translation = point - scaledPoint;

                std::vector<Vector3D> newPoints;
                std::vector<Face> newFaces;
                for (Vector3D point : figure.points) newPoints.push_back(point * scaleMatrix + translation);
                for (Face face : figure.faces) newFaces.push_back(face.clone());
                Figure scaled = Figure(newPoints, newFaces, figure.color);
                intermediate.push_back(scaled);
            }
        }

        fractal = intermediate;
    }
}

void generateMengerSponge(Figure& base, const int nr_iterations, const double scale) {
    Matrix scaleMatrix = transformations::scaleFigure(1.0 / 9.0);

    for (int i = 0; i < nr_iterations; i++) {
        Matrix scaleMatrix = transformations::scaleFigure(1.0 / 9.0);
        Figures3D figures;

        for (int j = 0; j < 9; j++) {
             std::vector<Vector3D> newPoints;
                std::vector<Face> newFaces;
                for (Vector3D point : base.points) newPoints.push_back(point * scaleMatrix);
                for (Face face : base.faces) newFaces.push_back(face.clone());
                Figure scaled = Figure(newPoints, newFaces, base.color);
                figures.push_back(scaled);
        }

       
    }
}