#include <cmath>

#include "../lib/vector3d/vector3d.h"
#include "Figure.h"

Matrix scaleFigure(const double scale) { 
    Matrix m;

    m(1, 1) = scale;
    m(2, 2) = scale;
    m(3, 3) = scale;

    return m;
}

Matrix rotateX(const double angle) { 
    Matrix m;

    m(2, 2) = std::cos(angle);
    m(3, 3) = std::cos(angle);
    m(2, 3) = std::sin(angle);
    m(3, 2) = -std::sin(angle);

    return m;
}

Matrix rotateY(const double angle) { 
    Matrix m;

    m(1, 1) = std::cos(angle);
    m(1, 3) = -std::sin(angle);
    m(3, 1) = std::sin(angle);
    m(3, 3) = std::cos(angle);

    return m;
}

Matrix rotateZ(const double angle) { 
    Matrix m;

    m(1, 1) = std::cos(angle);
    m(1, 2) = std::sin(angle);
    m(2, 1) = -std::sin(angle);
    m(2, 2) = std::cos(angle);

    return m;
}

Matrix translate(const Vector3D &vector) { 
    Matrix m;

    m(4, 1) = vector.x;
    m(4, 2) = vector.y;
    m(4, 3) = vector.z;

    return m;
}

void toPolar(const Vector3D &point, double &theta, double &phi, double &r) {
    r = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    theta = std::atan2(point.y, point.x);
    phi = std::acos(point.z / r);
}


Matrix eyePointTrans(const Vector3D &eyepoint) {
    double theta;
    double phi;
    double r;

    toPolar(eyepoint, theta, phi, r);

    Matrix v;
    v(1, 1) = -std::sin(theta);
    v(1, 2) = -std::cos(theta) * std::cos(phi);
    v(1, 3) = std::cos(theta) * std::sin(phi);

    v(2, 1) = std::cos(theta);
    v(2, 2) = -std::sin(theta) * std::cos(phi);
    v(2, 3) = std::sin(theta) * std::sin(phi);

    v(3, 2) = std::sin(phi);
    v(3, 3) = std::cos(phi);

    v(4, 3) = -r;

    return v;
}