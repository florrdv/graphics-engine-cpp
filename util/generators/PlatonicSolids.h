#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_GENERATORS_PLATONICSOLIDS_H_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_GENERATORS_PLATONICSOLIDS_H_

#include "../Figure.h"
#include "../Face.h"

namespace PlatonicSolids {
    Figure createDodecahedron(Color a);
    
    Figure createTetrahedron(Color a);

    Figure createCube(Color a) ;

    Figure createOctahedron(Color a);

    Figure createIcosahedron(Color a);

    Figure createTruncatedIcosahedron(Color a);

    Figure createCone(Color a, const int n, const double h);

    Figure createSphere(Color a, const double radius, const int n);

    Figure createCylinder(Color a, const int n, const double h);

    Figure createTorus(Color a, const double r, const double R, const int n, const int m);
}

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_GENERATORS_PLATONICSOLIDS_H_