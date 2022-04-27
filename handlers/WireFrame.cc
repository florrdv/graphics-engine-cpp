#include "WireFrame.h"

#include <fstream>
#include <math.h>

#include "Universal.h"
#include "3DLsystem.h"
#include "Fractals.h"
#include "../ini_configuration.h"
#include "../util/generators/PlatonicSolids.h"
#include "../util/generators/Transformations.h"

img::EasyImage wireFrame(const ini::Configuration& c, bool zBuffer) {
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

        Figures3D currentFigures;
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

                currentFigures.push_back(Figure(vectors, faces, color));

            }
        }
        else if (type == "Cube") currentFigures.push_back(PlatonicSolids::createCube(color));
        else if (type == "Tetrahedron") currentFigures.push_back(PlatonicSolids::createTetrahedron(color));
        else if (type == "Octahedron") currentFigures.push_back(PlatonicSolids::createOctahedron(color));
        else if (type == "Icosahedron") currentFigures.push_back(PlatonicSolids::createIcosahedron(color));
        else if (type == "Dodecahedron") currentFigures.push_back(PlatonicSolids::createDodecahedron(color));
        else if (type == "Torus") {
            double r;
            double R;
            int n;
            int m;

            if (!base["r"].as_double_if_exists(r)) std::cout << "⛔️| Failed to fetch r" << std::endl;
            if (!base["R"].as_double_if_exists(R)) std::cout << "⛔️| Failed to fetch R" << std::endl;
            if (!base["n"].as_int_if_exists(n)) std::cout << "⛔️| Failed to fetch n" << std::endl;
            if (!base["m"].as_int_if_exists(m)) std::cout << "⛔️| Failed to fetch m" << std::endl;

            currentFigures.push_back(PlatonicSolids::createTorus(color, r, R, n, m));
        } else if (type == "Sphere") {
            int n;

            if (!base["n"].as_int_if_exists(n)) std::cout << "⛔️| Failed to fetch n" << std::endl;
            currentFigures.push_back(PlatonicSolids::createSphere(color, 1, n));
        } else if (type == "Cylinder") {
            int n;
            double h;

            if (!base["n"].as_int_if_exists(n)) std::cout << "⛔️| Failed to fetch n" << std::endl;
            if (!base["height"].as_double_if_exists(h)) std::cout << "⛔️| Failed to fetch h" << std::endl;

            currentFigures.push_back(PlatonicSolids::createCylinder(color, n, h));
        }
        else if (type == "Cone") {
            int n;
            double h;

            if (!base["n"].as_int_if_exists(n)) std::cout << "⛔️| Failed to fetch n" << std::endl;
            if (!base["height"].as_double_if_exists(h)) std::cout << "⛔️| Failed to fetch h" << std::endl;

            currentFigures.push_back(PlatonicSolids::createCone(color, n, h));
        }
        else if (type == "3DLSystem") {
            Figure figure;
            std::string inputFile;
            if (!base["inputfile"].as_string_if_exists(inputFile)) std::cout << "⛔️| Failed to fetch # points" << std::endl;

            LParser::LSystem3D l_system;
            std::ifstream input_stream(inputFile);
            input_stream >> l_system;
            input_stream.close();

            figure.color = color;

            draw3DLSystem(l_system, figure, color);
            currentFigures.push_back(figure);
        }
        else if (type == "FractalCube") {
            int nrIterations;
            if (!base["nrIterations"].as_int_if_exists(nrIterations)) std::cout << "⛔️| Failed to fetch # iterations" << std::endl;

            double fractalScale;
            if (!base["fractalScale"].as_double_if_exists(fractalScale)) std::cout << "⛔️| Failed to fetch scale" << std::endl;

            Figure baseFig = PlatonicSolids::createCube(color);;
            generateFractal(baseFig, currentFigures, nrIterations, fractalScale);
        }
        else if (type == "FractalTetrahedron") {
            int nrIterations;
            if (!base["nrIterations"].as_int_if_exists(nrIterations)) std::cout << "⛔️| Failed to fetch # iterations" << std::endl;

            double fractalScale;
            if (!base["fractalScale"].as_double_if_exists(fractalScale)) std::cout << "⛔️| Failed to fetch scale" << std::endl;

            Figure baseFig = PlatonicSolids::createTetrahedron(color);;
            generateFractal(baseFig, currentFigures, nrIterations, fractalScale);
        }
        else if (type == "FractalIcosahedron") {
            int nrIterations;
            if (!base["nrIterations"].as_int_if_exists(nrIterations)) std::cout << "⛔️| Failed to fetch # iterations" << std::endl;

            double fractalScale;
            if (!base["fractalScale"].as_double_if_exists(fractalScale)) std::cout << "⛔️| Failed to fetch scale" << std::endl;

            Figure baseFig = PlatonicSolids::createIcosahedron(color);;
            generateFractal(baseFig, currentFigures, nrIterations, fractalScale);
        }
        else if (type == "FractalIcosahedron") {
            int nrIterations;
            if (!base["nrIterations"].as_int_if_exists(nrIterations)) std::cout << "⛔️| Failed to fetch # iterations" << std::endl;

            double fractalScale;
            if (!base["fractalScale"].as_double_if_exists(fractalScale)) std::cout << "⛔️| Failed to fetch scale" << std::endl;

            Figure baseFig = PlatonicSolids::createIcosahedron(color);;
            generateFractal(baseFig, currentFigures, nrIterations, fractalScale);
        }
        else if (type == "FractalOctahedron") {
            int nrIterations;
            if (!base["nrIterations"].as_int_if_exists(nrIterations)) std::cout << "⛔️| Failed to fetch # iterations" << std::endl;

            double fractalScale;
            if (!base["fractalScale"].as_double_if_exists(fractalScale)) std::cout << "⛔️| Failed to fetch scale" << std::endl;

            Figure baseFig = PlatonicSolids::createOctahedron(color);;
            generateFractal(baseFig, currentFigures, nrIterations, fractalScale);
        }
        else if (type == "FractalDodecahedron") {
            int nrIterations;
            if (!base["nrIterations"].as_int_if_exists(nrIterations)) std::cout << "⛔️| Failed to fetch # iterations" << std::endl;

            double fractalScale;
            if (!base["fractalScale"].as_double_if_exists(fractalScale)) std::cout << "⛔️| Failed to fetch scale" << std::endl;

            Figure baseFig = PlatonicSolids::createDodecahedron(color);;
            generateFractal(baseFig, currentFigures, nrIterations, fractalScale);
        }
        else if (type == "FractalBuckyBall") {
            int nrIterations;
            if (!base["nrIterations"].as_int_if_exists(nrIterations)) std::cout << "⛔️| Failed to fetch # iterations" << std::endl;

            double fractalScale;
            if (!base["fractalScale"].as_double_if_exists(fractalScale)) std::cout << "⛔️| Failed to fetch scale" << std::endl;

            Figure baseFig = PlatonicSolids::createTruncatedIcosahedron(color);;
            generateFractal(baseFig, currentFigures, nrIterations, fractalScale);
        }
        else if (type == "BuckyBall") currentFigures.push_back(PlatonicSolids::createTruncatedIcosahedron(color));
        else if (type == "MengerSponge") {
            int nrIterations;
            if (!base["nrIterations"].as_int_if_exists(nrIterations)) std::cout << "⛔️| Failed to fetch # iterations" << std::endl;

            Figure baseFig = PlatonicSolids::createCube(color);
            currentFigures.push_back(baseFig);
            generateMengerSponge(currentFigures, 0, nrIterations);
        }

        Matrix rotateMatrixX = transformations::rotateX(rotateX * M_PI / 180);
        Matrix rotateMatrixY = transformations::rotateY(rotateY * M_PI / 180);
        Matrix rotateMatrixZ = transformations::rotateZ(rotateZ * M_PI / 180);
        applyTransformationAll(currentFigures, rotateMatrixX);
        applyTransformationAll(currentFigures, rotateMatrixY);
        applyTransformationAll(currentFigures, rotateMatrixZ);

        Matrix scaleMatrix = transformations::scaleFigure(scale);
        applyTransformationAll(currentFigures, scaleMatrix);

        Matrix translateMatrix = transformations::translate(Vector3D::point(center[0], center[1], center[2]));
        applyTransformationAll(currentFigures, translateMatrix);

        figures.insert(figures.end(), currentFigures.begin(), currentFigures.end());

    }

    Matrix eyePointTransMatrix = transformations::eyePointTrans(eye);
    applyTransformationAll(figures, eyePointTransMatrix);

    Lines2D lines = projectAll(figures);
    return draw2DLines(lines, size, backgroundColor, zBuffer);
}