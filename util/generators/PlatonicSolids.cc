#include "../Figure.h"
#include "../Face.h"

Figure createTetrahedron() {
    Vector3D p0 = Vector3D::point(1, -1, -1);
    Vector3D p1 = Vector3D::point(-1, 1, -1);
    Vector3D p2 = Vector3D::point(1, 1, 1);
    Vector3D p3 = Vector3D::point(-1, -1, 1);

    Face f1 = Face({0, 1, 2});
    Face f2 = Face({1, 3, 2});
    Face f3 = Face({0, 3, 1});
    Face f4 = Face({0, 2, 3});
}