#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_GENERATORS_PLATONICSOLIDS_H_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_GENERATORS_PLATONICSOLIDS_H_

#include "../Figure.h"
#include "../Face.h"

namespace PlatonicSolids {
    Figure createDodecahedron(Color a, Color d);
    
    Figure createTetrahedron(Color a, Color d);

    Figure createCube(Color a, Color d) ;

    Figure createOctahedron(Color a, Color d);

    Figure createIcosahedron(Color a, Color d);

    Figure createTruncatedIcosahedron(Color a, Color d);

    Figure createCone(Color a, Color d, const int n, const double h);

    Figure createSphere(Color a, Color d, const double radius, const int n);

    Figure createCylinder(Color a, Color d, const int n, const double h);

    Figure createTorus(Color a, Color d, const double r, const double R, const int n, const int m);
}

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_GENERATORS_PLATONICSOLIDS_H_