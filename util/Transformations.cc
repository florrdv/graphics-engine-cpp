#include "../lib/vector3d/vector3d.h"

Matrix scaleFigure(const double scale) { 
    Matrix m;

    m(0, 0) = scale;
    m(1, 1) = scale;
    m(2, 2) = scale;

    return m;
}