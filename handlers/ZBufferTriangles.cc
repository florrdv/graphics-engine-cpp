#include "ZBufferTriangles.h"

#include "../util/generators/PlatonicSolids.h"
#include "../util/generators/Transformations.h"
#include "../lib/l_parser/l_parser.h"
#include "Universal.h"
#include <fstream>
#include <math.h>
#include <cmath>

void drawFigure(img::EasyImage &img, Figure &f, double size, Color &background) {
    Lines2D lines = projectFig(f);
    ImageDetails details = getImageDetails(lines, size);

    ZBuffer z = ZBuffer(std::lround(details.imageX), std::lround(details.imageY));

    f.triangulate();

    double d = 0.95 * details.imageX/details.xRange;

    double dcX = d * (details.xMin + details.xMax) / 2;
    double dcY = d * (details.yMin + details.yMax) / 2;
    double dX = details.imageX / 2 - dcX;
    double dY = details.imageY / 2 - dcY;

    for (Face face : f.faces) {
        draw_zbuf_triag(z, img, 
                        f.points[face.pointIndexes[0]], f.points[face.pointIndexes[1]], f.points[face.pointIndexes[2]],
                        d,
                        dX,
                        dY,
                        f.color);
    }

}

img::EasyImage drawFigures(Figures3D &figures, double size, Color &background) {
    Lines2D lines = projectAll(figures);
    ImageDetails details = getImageDetails(lines, size);

    ZBuffer z = ZBuffer(std::lround(details.imageX), std::lround(details.imageY));
    img::EasyImage img(details.imageX, details.imageY, background.toNative());

    for (Figure figure : figures) {
        drawFigure(img, figure, size, background);
    }

    return img;
}

img::EasyImage zBufferTriangle(const ini::Configuration& c) {
    Figures3D figures;

    int size;
    if (!c["General"]["size"].as_int_if_exists(size)) std::cout << "⛔️| Failed to fetch size" << std::endl;

    std::vector<int> eyeRaw;
    if (!c["General"]["eye"].as_int_tuple_if_exists(eyeRaw)) std::cout << "⛔️| Failed to read eye" << std::endl;
    Vector3D eye = Vector3D::point(eyeRaw[0], eyeRaw[1], eyeRaw[2]);

    std::vector<double> backgroundColorRaw;
    if (!c["General"]["backgroundcolor"].as_double_tuple_if_exists(backgroundColorRaw)) std::cout << "⛔️| Failed to fetch background color" << std::endl;
    Color backgroundColor = Color(backgroundColorRaw[0], backgroundColorRaw[1], backgroundColorRaw[2]);

    int nrFigures;
    if (!c["General"]["nrFigures"].as_int_if_exists(nrFigures)) std::cout << "⛔️| Failed to fetch # figures" << std::endl;

    for (int f = 0; f < nrFigures; f++) {
        auto base = c["Figure" + std::to_string(f)];


        std::vector<double> colorRaw;
        if (!base["color"].as_double_tuple_if_exists(colorRaw)) std::cout << "⛔️| Failed to fetch color" << std::endl;
        Color color = Color(colorRaw[0], colorRaw[1], colorRaw[2]);

        std::string type;
        if (!base["type"].as_string_if_exists(type)) std::cout << "⛔️| Failed to fetch type" << std::endl;

        double scale;
        std::vector<double> center;
        double rotateX;
        double rotateY;
        double rotateZ;
        if (!base["scale"].as_double_if_exists(scale)) std::cout << "⛔️| Failed to fetch scale" << std::endl;
        if (!base["center"].as_double_tuple_if_exists(center)) std::cout << "⛔️| Failed to fetch center" << std::endl;
        if (!base["rotateX"].as_double_if_exists(rotateX)) std::cout << "⛔️| Failed to fetch rotateX" << std::endl;
        if (!base["rotateY"].as_double_if_exists(rotateY)) std::cout << "⛔️| Failed to fetch rotateY" << std::endl;
        if (!base["rotateZ"].as_double_if_exists(rotateZ)) std::cout << "⛔️| Failed to fetch rotateZ" << std::endl;

        Figure figure;
        if (type == "LineDrawing") {
            int nrPoints;
            int nrLines;
            if (!base["nrPoints"].as_int_if_exists(nrPoints)) std::cout << "⛔️| Failed to fetch # points" << std::endl;
            if (!base["nrLines"].as_int_if_exists(nrLines)) std::cout << "⛔️| Failed to fetch # lines" << std::endl;

            // Read points
            std::vector<Vector3D> vectors;
            for (int i = 0; i < nrPoints; i++) {
                std::vector<double> p;
                auto f = "point" + std::to_string(i);
                if (!base[f].as_double_tuple_if_exists(p)) break;

                Vector3D vector = Vector3D::point(p[0], p[1], p[2]);
                vectors.push_back(vector);
            }

            // Read faces
            std::vector<Face> faces;
            for (int i = 0; i < nrLines; i++) {
                std::vector<int> l;
                auto f = "line" + std::to_string(i);
                if (!base[f].as_int_tuple_if_exists(l)) break;

                Face face = Face(l);
                faces.push_back(face);

                figure = Figure(vectors, faces, color);

            }
        }
        else if (type == "Cube") figure = PlatonicSolids::createCube(color);
        else if (type == "Tetrahedron") figure = PlatonicSolids::createTetrahedron(color);
        else if (type == "Octahedron") figure = PlatonicSolids::createOctahedron(color);
        else if (type == "Icosahedron") figure = PlatonicSolids::createIcosahedron(color);
        else if (type == "Dodecahedron") figure = PlatonicSolids::createDodecahedron(color);
        else if (type == "Torus") {
            double r;
            double R;
            int n;
            int m;

            if (!base["r"].as_double_if_exists(r)) std::cout << "⛔️| Failed to fetch r" << std::endl;
            if (!base["R"].as_double_if_exists(R)) std::cout << "⛔️| Failed to fetch R" << std::endl;
            if (!base["n"].as_int_if_exists(n)) std::cout << "⛔️| Failed to fetch n" << std::endl;
            if (!base["m"].as_int_if_exists(m)) std::cout << "⛔️| Failed to fetch m" << std::endl;

            figure = PlatonicSolids::createTorus(color, r, R, n, m);
        } else if (type == "Sphere") {
            int n;

            if (!base["n"].as_int_if_exists(n)) std::cout << "⛔️| Failed to fetch n" << std::endl;
            figure = PlatonicSolids::createSphere(color, 1, n);
        } else if (type == "Cylinder") {
            int n;
            double h;

            if (!base["n"].as_int_if_exists(n)) std::cout << "⛔️| Failed to fetch n" << std::endl;
            if (!base["height"].as_double_if_exists(h)) std::cout << "⛔️| Failed to fetch h" << std::endl;

            figure = PlatonicSolids::createCylinder(color, n, h);
        }
        else if (type == "Cone") {
            int n;
            double h;

            if (!base["n"].as_int_if_exists(n)) std::cout << "⛔️| Failed to fetch n" << std::endl;
            if (!base["height"].as_double_if_exists(h)) std::cout << "⛔️| Failed to fetch h" << std::endl;

            figure = PlatonicSolids::createCone(color, n, h);
        }


        Matrix rotateMatrixX = transformations::rotateX(rotateX * M_PI / 180);
        Matrix rotateMatrixY = transformations::rotateY(rotateY * M_PI / 180);
        Matrix rotateMatrixZ = transformations::rotateZ(rotateZ * M_PI / 180);
        applyTransformation(figure, rotateMatrixX);
        applyTransformation(figure, rotateMatrixY);
        applyTransformation(figure, rotateMatrixZ);

        Matrix scaleMatrix = transformations::scaleFigure(scale);
        applyTransformation(figure, scaleMatrix);

        Matrix translateMatrix = transformations::translate(Vector3D::point(center[0], center[1], center[2]));
        applyTransformation(figure, translateMatrix);

        figures.push_back(figure);

    }

    Matrix eyePointTransMatrix = transformations::eyePointTrans(eye);
    applyTransformationAll(figures, eyePointTransMatrix);

    return drawFigures(figures, size, backgroundColor);
}