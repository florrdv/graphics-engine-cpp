#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_GENERATORS_PLATONICSOLIDS_H_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_GENERATORS_PLATONICSOLIDS_H_

#include "../Figure.h"
#include "../Face.h"

namespace PlatonicSolids {
    Figure createTetrahedron(Color c);

    Figure createCube(Color c) ;

    Figure createOctahedron(Color c);

    Figure createIcosahedron(Color c);

    Figure createCone(Color c, const int n, const double h);


    Figure createSphere(Color c, const double radius, const int n);


    Figure createCylinder(Color c, const int n, const double h);

    Figure createTorus(Color c, const double r, const double R, const int n, const int m);
}

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_GENERATORS_PLATONICSOLIDS_H_