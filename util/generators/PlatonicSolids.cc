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

        Face f0 = Face({ 0, 1, 2 });
        Face f1 = Face({ 1, 3, 2 });
        Face f2 = Face({ 0, 3, 1 });
        Face f3 = Face({ 0, 2, 3 });

        return Figure({ p0, p1, p2, p3 }, { f0, f1, f2, f3 }, c);
    }
}
