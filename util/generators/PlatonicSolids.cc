#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_GENERATORS_PLATONICSOLIDS_CC_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_GENERATORS_PLATONICSOLIDS_CC_

#include <cmath>

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

    Figure createSphere(Color c, const double radius, const int n) {
        Figure intermediate = createTetrahedron(c);

        for (int i = 0; i < n; i++) {
            std::vector<Face> newFaces;
            for (auto face : intermediate.faces) {
                int aIndex = face.pointIndexes[0];
                int bIndex = face.pointIndexes[1];
                int cIndex = face.pointIndexes[2];
                Vector3D a = intermediate.points[face.pointIndexes[0]];
                Vector3D b = intermediate.points[face.pointIndexes[1]];
                Vector3D c = intermediate.points[face.pointIndexes[2]];

                Vector3D d = (a + b) / 2.0;
                Vector3D e = (a + c) / 2.0;
                Vector3D f = (b + c) / 2.0;

                int dIndex = intermediate.points.size();
                int eIndex = intermediate.points.size();
                int fIndex = intermediate.points.size();
                intermediate.points.push_back(d);
                intermediate.points.push_back(e);
                intermediate.points.push_back(f);

                newFaces.push_back(Face({ aIndex, dIndex, eIndex }));
                newFaces.push_back(Face({ bIndex, fIndex, dIndex }));
                newFaces.push_back(Face({ cIndex, eIndex, fIndex }));
                newFaces.push_back(Face({ dIndex, fIndex, eIndex }));
            }

            intermediate.faces = newFaces;
        }

        for (Vector3D &point : intermediate.points) { 
            double x = point.x;
            double y = point.y;
            double z = point.z;
            double r = std::sqrt(x * x +  y*y + z*z);
            point.x = x / r;
            point.y = y / r;
            point.z = z / r;
        }

        return intermediate;
    }
}

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_GENERATORS_PLATONICSOLIDS_CC_
