#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_GENERATORS_TRANSFORMATIONS_H_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_GENERATORS_TRANSFORMATIONS_H_

#include "../Figure.h"

namespace transformations {
    Matrix scaleFigure(const double scale);

    Matrix rotateX(const double angle);

    Matrix rotateY(const double angle);

    Matrix rotateZ(const double angle);

    Matrix translate(const Vector3D& vector);

    void toPolar(const Vector3D& point, double& theta, double& phi, double& r);


    Matrix eyePointTrans(const Vector3D& eyepoint);
}

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_GENERATORS_TRANSFORMATIONS_H_