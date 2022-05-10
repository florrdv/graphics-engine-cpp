#include "PlatonicSolids.h"
#include "../../handlers/Universal.h"
#include "Transformations.h"

#include <cmath>
#include <map>
#include <algorithm>
#include <math.h>

namespace PlatonicSolids {
    Figure createTetrahedron(Color a, Color d) {
        Vector3D p0 = Vector3D::point(1, -1, -1);
        Vector3D p1 = Vector3D::point(-1, 1, -1);
        Vector3D p2 = Vector3D::point(1, 1, 1);
        Vector3D p3 = Vector3D::point(-1, -1, 1);

        Face f0 = Face({ 0, 1, 2 });
        Face f1 = Face({ 1, 3, 2 });
        Face f2 = Face({ 0, 3, 1 });
        Face f3 = Face({ 0, 2, 3 });

        return Figure({ p0, p1, p2, p3 }, { f0, f1, f2, f3 }, a, d);
    }

    Figure createCube(Color a, Color d) {
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

        return Figure({ p0, p1, p2, p3, p4, p5, p6, p7 }, { f0, f1, f2, f3, f4, f5 }, a, d);
    }

    Figure createOctahedron(Color a, Color d) {
        Vector3D p0 = Vector3D::point(1, 0, 0);
        Vector3D p1 = Vector3D::point(0, 1, 0);
        Vector3D p2 = Vector3D::point(-1, 0, 0);
        Vector3D p3 = Vector3D::point(0, -1, 0);

        Vector3D p4 = Vector3D::point(0, 0, -1);
        Vector3D p5 = Vector3D::point(0, 0, 1);

        Face f0 = Face({ 0, 1, 5 });
        Face f1 = Face({ 1, 2, 5 });
        Face f2 = Face({ 2, 3, 5 });
        Face f3 = Face({ 3, 0, 5 });

        Face f4 = Face({ 1, 0, 4 });
        Face f5 = Face({ 2, 1, 4 });
        Face f6 = Face({ 3, 2, 4 });
        Face f7 = Face({ 0, 3, 4 });

        return Figure({ p0, p1, p2, p3, p4, p5 }, { f0, f1, f2, f3, f4, f5, f6, f7 }, a, d);
    }

    Figure createIcosahedron(Color a, Color d) {
        Vector3D p0  = Vector3D::point(0, 0, sqrt(5)/2);

        Vector3D p1  = Vector3D::point(std::cos((2 - 2)*2*M_PI/5), std::sin((2 - 2)*2*M_PI/5), 0.5);
        Vector3D p2  = Vector3D::point(std::cos((3 - 2)*2*M_PI/5), std::sin((3 - 2)*2*M_PI/5), 0.5);
        Vector3D p3  = Vector3D::point(std::cos((4 - 2)*2*M_PI/5), std::sin((4 - 2)*2*M_PI/5), 0.5);
        Vector3D p4  = Vector3D::point(std::cos((5 - 2)*2*M_PI/5), std::sin((5 - 2)*2*M_PI/5), 0.5);
        Vector3D p5  = Vector3D::point(std::cos((6 - 2)*2*M_PI/5), std::sin((6 - 2)*2*M_PI/5), 0.5);

        Vector3D p6  = Vector3D::point(std::cos(M_PI/5+(7 -7)*2*M_PI/5), std::sin(M_PI/5+(7 -7)*2*M_PI/5), -0.5);
        Vector3D p7  = Vector3D::point(std::cos(M_PI/5+(8 -7)*2*M_PI/5), std::sin(M_PI/5+(8 -7)*2*M_PI/5), -0.5);
        Vector3D p8  = Vector3D::point(std::cos(M_PI/5+(9 -7)*2*M_PI/5), std::sin(M_PI/5+(9 -7)*2*M_PI/5), -0.5);
        Vector3D p9  = Vector3D::point(std::cos(M_PI/5+(10-7)*2*M_PI/5), std::sin(M_PI/5+(10-7)*2*M_PI/5), -0.5);
        Vector3D p10 = Vector3D::point(std::cos(M_PI/5+(11-7)*2*M_PI/5), std::sin(M_PI/5+(11-7)*2*M_PI/5), -0.5);

        Vector3D p11 = Vector3D::point(0, 0, -sqrt(5)/2);

        Face f0 = Face({ 0, 1,  2 });
        Face f1 = Face({ 0, 2,  3 });
        Face f2 = Face({ 0, 3,  4 });
        Face f3 = Face({ 0, 4,  5 });

        Face f4 = Face({ 0, 5,  1 });
        Face f5 = Face({ 1, 6,  2 });
        Face f6 = Face({ 2, 6,  7 });
        Face f7 = Face({ 2, 7,  3 });

        Face f8 = Face({ 3, 7,  8 });
        Face f9 = Face({ 3, 8,  4 });
        Face f10 = Face({ 4, 8,  9 });
        Face f11 = Face({ 4, 9,  5 });

        Face f12 = Face({ 5, 9,  10 });
        Face f13 = Face({ 5, 10, 1 });
        Face f14 = Face({ 1, 10, 6 });
        Face f15 = Face({ 11, 7, 6 });

        Face f16 = Face({ 11, 8, 7 });
        Face f17 = Face({ 11, 9, 8 });
        Face f18 = Face({ 11, 10, 9 });
        Face f19 = Face({ 11, 6, 10 });

        return Figure({ p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11 }, { f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14, f15, f16, f17, f18, f19}, a, d);
    }

    Figure createDodecahedron(Color a, Color d) {
        Figure ico = createIcosahedron(a, d);

        std::vector<Vector3D> p;
        for (auto face : ico.faces) { 
            Vector3D p0 = ico.points[face.pointIndexes[0]];
            Vector3D p1 = ico.points[face.pointIndexes[1]];
            Vector3D p2 = ico.points[face.pointIndexes[2]];

            Vector3D sum = p0 + p1 + p2;
            Vector3D constructed = Vector3D::point(sum.x / 3, sum.y / 3, sum.z / 3);
            p.push_back(constructed);
        }

        Face f0 = Face({ 0, 1, 2, 3, 4 });
        Face f1 = Face({ 0, 5, 6, 7, 1 });
        Face f2 = Face({ 1, 7, 8, 9, 2 });
        Face f3 = Face({ 2, 9, 10, 11, 3 });

        Face f4 = Face({ 3, 11, 12, 13, 4 });
        Face f5 = Face({ 4, 13, 14, 5, 0 });
        Face f6 = Face({ 19, 18, 17, 16, 15 });
        Face f7 = Face({ 19, 14, 13, 12, 18 });

        Face f8 = Face({ 18, 12, 11, 10, 17 });
        Face f9 = Face({ 17, 10, 9, 8, 16 });
        Face f10 = Face({ 16, 8, 7, 6, 15 });
        Face f11 = Face({ 15, 6, 5, 14, 19 });

        return Figure(p, { f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11 }, a, d);
    }

    Vector3D calculateTriangleCentroid(Figure &ico, Face &face) {
        Vector3D p0 = ico.points[face.pointIndexes[0]];
        Vector3D p1 = ico.points[face.pointIndexes[1]];
        Vector3D p2 = ico.points[face.pointIndexes[2]];

        Vector3D sum = p0 + p1 + p2;
        Vector3D centroid = Vector3D::point(sum.x / 3, sum.y / 3, sum.z / 3);

        return centroid;
    }

    // Inspired by: https://github.com/kovacsv/JSModeler
    Figure createTruncatedIcosahedron(Color ambient, Color diffuse) {
        // Cartesian coordinates for the vertices of a truncated icosahedron centered at the origin are all even permutations of:
        // (0, ±1, ±3φ)
        // (±1, ±(2 + φ), ±2φ)
        // (±φ, ±2, ±(2φ + 1))
        // where φ = (1 + √5) / 2 is the golden mean
        // Source: https://en.wikipedia.org/wiki/Truncated_icosahedron

	    double goldenMean = (1.0 + sqrt(5.0)) / 2.0;
	    double a = 2.0 * goldenMean;
	    double b = 3.0 * goldenMean;
	    double c = 1.0 + a;
	    double d = 2.0 + goldenMean;

        std::vector<Vector3D> points = {
            Vector3D::point(+0, +1.0, +b),
            Vector3D::point(+0, +1.0, -b),
            Vector3D::point(+0, -1.0, +b),
            Vector3D::point(+0, -1.0, -b),

            Vector3D::point(+1.0, +b, +0),
            Vector3D::point(+1.0, -b, +0),
            Vector3D::point(-1.0, +b, +0),
            Vector3D::point(-1.0, -b, +0),

            Vector3D::point(+b, +0, +1.0),
            Vector3D::point(-b, +0, +1.0),
            Vector3D::point(+b, +0, -1.0),
            Vector3D::point(-b, +0, -1.0),

            Vector3D::point(+2.0, +c, +goldenMean),
            Vector3D::point(+2.0, +c, -goldenMean),
            Vector3D::point(+2.0, -c, +goldenMean),
            Vector3D::point(-2.0, +c, +goldenMean),
            Vector3D::point(+2.0, -c, -goldenMean),
            Vector3D::point(-2.0, +c, -goldenMean),
            Vector3D::point(-2.0, -c, +goldenMean),
            Vector3D::point(-2.0, -c, -goldenMean),

            Vector3D::point(+c, +goldenMean, +2.0),
            Vector3D::point(+c, -goldenMean, +2.0),
            Vector3D::point(-c, +goldenMean, +2.0),
            Vector3D::point(+c, +goldenMean, -2.0),
            Vector3D::point(-c, -goldenMean, +2.0),
            Vector3D::point(+c, -goldenMean, -2.0),
            Vector3D::point(-c, +goldenMean, -2.0),
            Vector3D::point(-c, -goldenMean, -2.0),

            Vector3D::point(+goldenMean, +2.0, +c),
            Vector3D::point(-goldenMean, +2.0, +c),
            Vector3D::point(+goldenMean, +2.0, -c),
            Vector3D::point(+goldenMean, -2.0, +c),
            Vector3D::point(-goldenMean, +2.0, -c),
            Vector3D::point(-goldenMean, -2.0, +c),
            Vector3D::point(+goldenMean, -2.0, -c),
            Vector3D::point(-goldenMean, -2.0, -c),

            Vector3D::point(+1.0, +d, +a),
            Vector3D::point(+1.0, +d, -a),
            Vector3D::point(+1.0, -d, +a),
            Vector3D::point(-1.0, +d, +a),
            Vector3D::point(+1.0, -d, -a),
            Vector3D::point(-1.0, +d, -a),
            Vector3D::point(-1.0, -d, +a),
            Vector3D::point(-1.0, -d, -a),

            Vector3D::point(+d, +a, +1.0),
            Vector3D::point(+d, -a, +1.0),
            Vector3D::point(-d, +a, +1.0),
            Vector3D::point(+d, +a, -1.0),
            Vector3D::point(-d, -a, +1.0),
            Vector3D::point(+d, -a, -1.0),
            Vector3D::point(-d, +a, -1.0),
            Vector3D::point(-d, -a, -1.0),

            Vector3D::point(+a, +1.0, +d),
            Vector3D::point(-a, +1.0, +d),
            Vector3D::point(+a, +1.0, -d),
            Vector3D::point(+a, -1.0, +d),
            Vector3D::point(-a, +1.0, -d),
            Vector3D::point(-a, -1.0, +d),
            Vector3D::point(+a, -1.0, -d),
            Vector3D::point(-a, -1.0, -d)
        };

        std::vector<Face> faces = {
            // Pentagons
            Face({0, 28, 36, 39, 29}),
            Face({1, 32, 41, 37, 30}),
            Face({2, 33, 42, 38, 31}),
            Face({3, 34, 40, 43, 35}),
            Face({4, 12, 44, 47, 13}),
            Face({5, 16, 49, 45, 14}),
            Face({6, 17, 50, 46, 15}),
            Face({7, 18, 48, 51, 19}),
            Face({8, 20, 52, 55, 21}),
            Face({9, 24, 57, 53, 22}),
            Face({10, 25, 58, 54, 23}),
            Face({11, 26, 56, 59, 27}),

            // Hexagons
            Face({0, 2, 31, 55, 52, 28}),
            Face({0, 29, 53, 57, 33, 2}),
            Face({1, 3, 35, 59, 56, 32}),
            Face({1, 30, 54, 58, 34, 3}),
            Face({4, 6, 15, 39, 36, 12}),
            Face({4, 13, 37, 41, 17, 6}),
            Face({5, 7, 19, 43, 40, 16}),
            Face({5, 14, 38, 42, 18, 7}),
            Face({8, 10, 23, 47, 44, 20}),
            Face({8, 21, 45, 49, 25, 10}),
            Face({9, 11, 27, 51, 48, 24}),
            Face({9, 22, 46, 50, 26, 11}),
            Face({12, 36, 28, 52, 20, 44}),
            Face({13, 47, 23, 54, 30, 37}),
            Face({14, 45, 21, 55, 31, 38}),
            Face({15, 46, 22, 53, 29, 39}),
            Face({16, 40, 34, 58, 25, 49}),
            Face({17, 41, 32, 56, 26, 50}),
            Face({18, 42, 33, 57, 24, 48}),
            Face({19, 51, 27, 59, 35, 43})
        };

        Figure figure = Figure(points, faces, ambient, diffuse);
        return figure;
    }

    Figure createCone(Color a, Color d, const int n, const double h) {
        // Points
        std::vector<Vector3D> points;
        for (int i = 0; i < n; i++) {
            Vector3D p = Vector3D::point(std::cos(2*i*M_PI/n), std::sin(2*i*M_PI/n), 0);
            points.push_back(p);
        }

        points.push_back(Vector3D::point(0, 0, h));

        // Faces
        std::vector<Face> faces;
        for (int i = 0; i < n; i++) {
            Face f = Face({ i, (i + 1) % n, n });
            faces.push_back(f);
        }

        // faces.push_back(Face({ n - 1, n-2, 0 }));

        return Figure(points, faces, a, d);
    }


    Figure createSphere(Color a, Color d, const double radius, const int n) {
        Figure intermediate = createIcosahedron(a, d);

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
                int eIndex = intermediate.points.size() + 1;
                int fIndex = intermediate.points.size() + 2;
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

        for (Vector3D& point : intermediate.points) {
            double x = point.x;
            double y = point.y;
            double z = point.z;
            double r = std::sqrt(x * x + y * y + z * z);
            point.x = x / r;
            point.y = y / r;
            point.z = z / r;
        }

        return intermediate;
    }


    Figure createCylinder(Color a, Color d, const int n, const double h) {
        // Points
        std::vector<Vector3D> points;
        for (int i = 0; i < n; i++) {
            Vector3D p = Vector3D::point(std::cos(2*i*M_PI/n), std::sin(2*i*M_PI/n), 0);
            points.push_back(p);
        }

        for (int i = 0; i < n; i++) {
            Vector3D p = Vector3D::point(std::cos(2*i*M_PI/n), std::sin(2*i*M_PI/n), h);
            points.push_back(p);
        }


        // Faces
        std::vector<Face> faces;
        for (int i = 0; i < n; i++) {
            Face f = Face({ i, (i + 1) % n, n + (i + 1) % n, n + i});
            faces.push_back(f);
        }

        std::vector<int> topPoints;
        std::vector<int> bottomPoints;

        for (int i = 0; i < n; i++) {
            topPoints.push_back(i);
            bottomPoints.push_back(n + i);
        }

        faces.push_back(Face(topPoints));
        faces.push_back(Face(bottomPoints));

        return Figure(points, faces, a, d);
    }

    Figure createTorus(Color a, Color d, const double r, const double R, const int n, const int m) {
        std::vector<Vector3D> points;

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                double u = 2 * i * M_PI / n;
                double v = 2 * j * M_PI / m;

                double x = (R + r * std::cos(v)) * std::cos(u);
                double y = (R + r * std::cos(v)) * std::sin(u);
                double z = r * std::sin(v);

                points.push_back(Vector3D::point(x, y, z));
            }
        }

        std::vector<Face> faces;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                Face f = Face({ i           + j             * m,
                                (i+1) % n   + j             * m,
                                (i+1) % n   + (j + 1) % m   * m,
                                i           + (j + 1) % m   * m
                            });

                faces.push_back(f);
            }
        }

        return Figure(points, faces, a, d);
    }
}
