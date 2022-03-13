#include <cmath>

#include "../lib/vector3d/vector3d.h"

Matrix scaleFigure(const double scale) { 
    Matrix m;

    m(0, 0) = scale;
    m(1, 1) = scale;
    m(2, 2) = scale;

    return m;
}

Matrix rotateX(const double angle) { 
    Matrix m;

    m(1, 1) = std::cos(angle);
    m(2, 2) = std::cos(angle);
    m(1, 2) = std::sin(angle);
    m(2, 1) = -std::sin(angle);

    return m;
}

Matrix rotateY(const double angle) { 
    Matrix m;

    m(0, 0) = std::cos(angle);
    m(0, 2) = -std::sin(angle);
    m(2, 0) = std::sin(angle);
    m(2, 2) = std::cos(angle);

    return m;
}