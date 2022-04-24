#include "Fractals.h"
#include "../util/generators/Transformations.h"

void generateFractal(Figure& fig, Figures3D& fractal, const int nr_iterations, const double scale) {
    Figure base = fig;
    Matrix scaleMatrix = transformations::scaleFigure(1 / scale);

    // Go through all iterations
    for (int i = 0; i < nr_iterations; i++) {
        for (Figure figure : fractal) {
            for (Vector3D point : figure.points) {
                std::vector<Vector3D> newPoints;
                std::vector<Face> newFaces;
                for (Vector3D point : figure.points) newPoints.push_back(point * scaleMatrix);
                for (Face face : figure.faces) newFaces.push_back(face.clone());
                Figure scaled = Figure(newPoints, newFaces, figure.color);
                fractal.push_back(scaled);
            }
        }
    }
}

img::EasyImage drawFractal(const ini::Configuration& c) {
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
        if (type == "FractalCube") 
        


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

    Lines2D lines = projectAll(figures);
    return draw2DLines(lines, size, backgroundColor, zBuffer);
}