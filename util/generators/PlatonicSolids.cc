#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_GENERATORS_PLATONICSOLIDS_CC_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_GENERATORS_PLATONICSOLIDS_CC_

#include "../Figure.h"
#include "../Face.h"

namespace PlatonicSolids {
    Figure createTetrahedron(Color c) {
        Vector3D p0 = Vector3D::point(1, -1, -1);
        Vector3D p1 = Vector3D::point(-1, 1, -1);
        Vector3D p2 = Vector3D::point(1, 1, 1);
        Vector3D p3 = Vector3D::point(-1, -1, 1);

        Face f0 = Face({ 0, 1, 2 });
        Face f1 = Face({ 1, 3, 2 });
        Face f2 = Face({ 0, 3, 1 });
        Face f3 = Face({ 0, 2, 3 });

        return Figure({ p0, p1, p2, p3 }, { f0, f1, f2, f3 }, c);
    }

    Figure createCube(Color c) {
        Vector3D p0 = Vector3D::point(1, -1, -1);
        Vector3D p1 = Vector3D::point(-1, 1, -1);
        Vector3D p2 = Vector3D::point(1, 1, 1);
        Vector3D p3 = Vector3D::point(-1, -1, 1);

        Vector3D p4 = Vector3D::point(1, 1, -1);
        Vector3D p5 = Vector3D::point(-1, -1, -1);
        Vector3D p6 = Vector3D::point(1, -1, 1);
        Vector3D p7 = Vector3D::point(-1, 1, 1);

        Face f0 = Face({ 0, 4, 2, 6 });
        Face f1 = Face({ 4, 1, 7, 2 });
        Face f2 = Face({ 1, 5, 3, 7 });
        Face f3 = Face({ 5, 0, 6, 3 });
        Face f4 = Face({ 6, 2, 7, 3 });
        Face f5 = Face({ 0, 5, 1, 4 });

        return Figure({ p0, p1, p2, p3, p4, p5, p6, p7 }, { f0, f1, f2, f3 }, c);
    }
}

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_GENERATORS_PLATONICSOLIDS_CC_
