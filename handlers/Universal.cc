#include "Universal.h"

#include <cmath>
#include <list>
#include <algorithm>
#include <limits>
#include <fstream>

#include "Fractals.h"
#include "3DLsystem.h"
#include "../lib/l_parser/l_parser.h"
#include "../util/Line2D.h"
#include "../util/Figure.h"
#include "../util/Light.h"
#include "../util/generators/Transformations.h"
#include "../util/generators/PlatonicSolids.h"

void applyTransformation(Figure& fig, const Matrix& m) {
    for (auto& p : fig.points) p *= m;
}

void applyTransformationAll(Figures3D& figs, const Matrix& m) {
    for (auto& f : figs) applyTransformation(f, m);
}

ImageDetails getImageDetails(const Lines2D &lines, const double size) {
    // TODO: handle edge case
    Line2D first = lines.front();

    double xMin = first.p1.x;
    double xMax = first.p1.x;

    double yMin = first.p1.y;
    double yMax = first.p1.y;

    // Determine min and max
    for (const Line2D& line : lines) {
        if (line.p1.x < xMin) xMin = line.p1.x;
        if (line.p2.x < xMin) xMin = line.p2.x;
        if (line.p1.x > xMax) xMax = line.p1.x;
        if (line.p2.x > xMax) xMax = line.p2.x;

        if (line.p1.y < yMin) yMin = line.p1.y;
        if (line.p2.y < yMin) yMin = line.p2.y;
        if (line.p1.y > yMax) yMax = line.p1.y;
        if (line.p2.y > yMax) yMax = line.p2.y;
    }

    // Compute variables needed for next step
    double xRange = std::abs(xMax - xMin);
    double yRange = std::abs(yMax - yMin);

    double imageX = size * xRange / std::max(xRange, yRange);
    double imageY = size * yRange / std::max(xRange, yRange);

    return ImageDetails{ 
        .imageX = imageX, 
        .imageY = imageY, 
        .xRange = xRange, 
        .yRange = yRange, 
        .xMin = xMin, 
        .xMax = xMax, 
        .yMin = yMin, 
        .yMax = yMax,
    };
}

void draw_zbuf_triag(ZBuffer &z, img::EasyImage &img, 
                    Vector3D const& A, Vector3D const& B, Vector3D const& C, 
                    double d, double dx, double dy, 
                    Color ambientReflection, Color diffuseReflection, Color specularReflection, double reflectionCoeff,
                    Lights3D& lights) {
    // Backwards compatibility
    if (lights.empty()) lights.push_back(Light(Color(1, 1, 1), Color(0, 0, 0), Color(0, 0, 0)));
    
    // Previous coordinates
    double xA = A.x;
    double yA = A.y;
    double zA = A.z;

    double xB = B.x;
    double yB = B.y;
    double zB = B.z;

    double xC = C.x;
    double yC = C.y;
    double zC = C.z;

    // New points
    double nxA = d*xA/-zA + dx;
    double nyA = d*yA/-zA + dy;
    Point2D nA = Point2D(nxA, nyA);

    double nxB = d*xB/-zB + dx;
    double nyB = d*yB/-zB + dy;
    Point2D nB = Point2D(nxB, nyB);

    double nxC = d*xC/-zC + dx;
    double nyC = d*yC/-zC + dy;
    Point2D nC = Point2D(nxC, nyC);

    int yMin = std::round(std::min({nA.y, nB.y, nC.y}) + 0.5);
    int yMax = std::round(std::max({nA.y, nB.y, nC.y}) - 0.5);

    // Calculate 1/zG
    double xG = (nA.x+nB.x+nC.x)/3;
    double yG = (nA.y+nB.y+nC.y)/3;
    
    double zG = 1/(3*zA) + 1/(3*zB) + 1/(3*zC);

    Vector3D u = B - A;
    double u1 = u.x;
    double u2 = u.y;
    double u3 = u.z;

    Vector3D v = C - A;
    double v1 = v.x;
    double v2 = v.y;
    double v3 = v.z;

    double w1 = u2*v3-u3*v2;
    double w2 = u3*v1-u1*v3;
    double w3 = u1*v2-u2*v1;

    double k = w1*xA + w2*yA + w3*zA;
    double dzdx = w1 / (-d*k);
    double dzdy = w2 / (-d*k);

    // Handle ambient light
    Color ambientResult = Color(0, 0, 0);
    for (Light& light : lights) ambientResult += light.ambientLight * ambientReflection;

    Color color = ambientResult;

    for (int yI = yMin; yI <= yMax; yI++) {
        // Determining xMin(xL) and XMax(xR)
        double xMinAB = std::numeric_limits<double>::infinity();
        double xMinAC = std::numeric_limits<double>::infinity();
        double xMinBC = std::numeric_limits<double>::infinity();

        double xMaxAB = -std::numeric_limits<double>::infinity();
        double xMaxAC = -std::numeric_limits<double>::infinity();
        double xMaxBC = -std::numeric_limits<double>::infinity();

        Point2D p;
        Point2D q;

        // AB
        p = nA;
        q = nB;
        if ((yI - p.y)*(yI - q.y) <= 0 && p.y != q.y) {
            double xI = q.x + (p.x - q.x)*(yI-q.y)/(p.y-q.y);
            xMinAB = xI; 
            xMaxAB = xI; 
        }

        // AC
        p = nA;
        q = nC;
        if ((yI - p.y)*(yI - q.y) <= 0 && p.y != q.y) {
            double xI = q.x + (p.x - q.x)*(yI-q.y)/(p.y-q.y);
            xMinAC = xI; 
            xMaxAC = xI; 
        }

        // BC
        p = nB;
        q = nC;
        if ((yI - p.y)*(yI - q.y) <= 0 && p.y != q.y) {
            double xI = q.x + (p.x - q.x)*(yI-q.y)/(p.y-q.y);
            xMinBC = xI; 
            xMaxBC = xI; 
        }

        int xL = std::lround(std::min({xMinAB, xMinAC, xMinBC}) + 0.5);
        int xR = std::lround(std::max({xMaxAB, xMaxAC, xMaxBC}) - 0.5);

        // zIndex preparation
        for (int xI = xL; xI <= xR; xI++) {
            // Calculate actual zIndex
            double zIndex = 1.0001 * zG + (xI-xG) * dzdx + (yI-yG) * dzdy;
            double previousValue = z[xI][yI];
            if (zIndex < previousValue) {
                z[xI][yI] = zIndex;
                img(xI, yI) = color.toNative();
            }
        }
    }
}

img::EasyImage draw2DLines(const Lines2D &lines, const int size, Color background, bool zBuffer) {
    ImageDetails details = getImageDetails(lines, (double) size);

    double d = 0.95 * details.imageX / details.xRange;
    double dcX = d * (details.xMin + details.xMax) / 2;
    double dcY = d * (details.yMin + details.yMax) / 2;
    double dX = details.imageX / 2 - dcX;
    double dY = details.imageY / 2 - dcY;

    // Create image
    img::EasyImage img(std::lround(details.imageX), std::lround(details.imageY), background.toNative());

    ZBuffer z = ZBuffer(std::lround(details.imageX), std::lround(details.imageY));

    // Re-position points
    for (Line2D line : lines) {
        line.p1.x *= d;
        line.p1.y *= d;

        line.p1.x += dX;
        line.p1.y += dY;

        line.p1.x = std::lround(line.p1.x);
        line.p1.y = std::lround(line.p1.y);

        line.p2.x *= d;
        line.p2.y *= d;

        line.p2.x += dX;
        line.p2.y += dY;

        line.p2.x = std::lround(line.p2.x);
        line.p2.y = std::lround(line.p2.y);

        if (zBuffer) {
            img.draw_zbuf_line(z, line.p1.x, line.p1.y, line.z1, line.p2.x, line.p2.y, line.z2, line.color.toNative());
        } else img.draw_line(line.p1.x, line.p1.y, line.p2.x, line.p2.y, line.color.toNative());
    }

    return img;
}

Point2D projectPoint(const Vector3D& point, const double d) {
    double x = d * point.x / -point.z;
    double y = d * point.y / -point.z;

    return Point2D(x, y);
}

Lines2D projectFig(const Figure& fig) {
    Lines2D lines;

    for (auto face : fig.faces) {
        for (unsigned int i = 0; i < face.pointIndexes.size(); i++) {
            Vector3D p1 = fig.points[face.pointIndexes[i]];

            Vector3D p2;
            if (i + 1 >= face.pointIndexes.size()) p2 = fig.points[face.pointIndexes[0]];
            else p2 = fig.points[face.pointIndexes[i + 1]];
            Line2D line = Line2D(projectPoint(p1, 1.0), projectPoint(p2, 1.0), p1.z, p2.z, fig.ambientReflection);

            lines.push_back(line);
        }
    }

    return lines;
}

Lines2D projectAll(const Figures3D& figs) {
    Lines2D lines;

    for (auto fig : figs) {
        Lines2D figLines = projectFig(fig);
        lines.insert(lines.end(), figLines.begin(), figLines.end());
    }

    return lines;
}

Figures3D parseLights(const ini::Configuration& c) {
    Lights3D;
}

Figures3D parseFigures(const ini::Configuration& c) {
    Figures3D figures;

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

            figure.ambientReflection = color;

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

    return figures;
}



Details parseGeneralDetails(const ini::Configuration& c) {
    int size;
    if (!c["General"]["size"].as_int_if_exists(size)) std::cout << "⛔️| Failed to fetch size" << std::endl;

    std::vector<int> eyeRaw;
    if (!c["General"]["eye"].as_int_tuple_if_exists(eyeRaw)) std::cout << "⛔️| Failed to read eye" << std::endl;
    Vector3D eye = Vector3D::point(eyeRaw[0], eyeRaw[1], eyeRaw[2]);

    std::vector<double> backgroundColorRaw;
    if (!c["General"]["backgroundcolor"].as_double_tuple_if_exists(backgroundColorRaw)) std::cout << "⛔️| Failed to fetch background color" << std::endl;
    Color backgroundColor = Color(backgroundColorRaw[0], backgroundColorRaw[1], backgroundColorRaw[2]);

    return Details { size, eye, backgroundColor };
}