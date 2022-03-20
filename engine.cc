#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <list>
#include <cmath>
#include <stack>
#include <math.h>

#include "easy_image.h"
#include "ini_configuration.h"

#include "util/Line2D.h"
#include "util/Point2D.h"
#include "util/Color.h"
#include "util/Figure.h"
#include "util/Face.h"
#include "util/generators/Transformations.cc"
#include "util/generators/PlatonicSolids.cc"

#include "lib/l_parser/l_parser.h"


using Lines2D = std::list<Line2D>;
using Figures3D = std::list<Figure>;

void applyTransformation(Figure& fig, const Matrix& m) {
    for (auto& p : fig.points) p *= m;
}

void applyTransformationAll(Figures3D& figs, const Matrix& m) {
    for (auto& f : figs) applyTransformation(f, m);
}

img::EasyImage draw2DLines(const Lines2D& lines, const int size, Color background) {
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

    double d = 0.95 * imageX / xRange;
    double dcX = d * (xMin + xMax) / 2;
    double dcY = d * (yMin + yMax) / 2;
    double dX = imageX / 2 - dcX;
    double dY = imageY / 2 - dcY;

    // Create image
    img::EasyImage img(std::lround(imageX), std::lround(imageY), background.toNative());

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

        img.draw_line(line.p1.x, line.p1.y, line.p2.x, line.p2.y, line.color.toNative());
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
        for (int i = 0; i < face.pointIndexes.size(); i++) {
            Vector3D p1 = fig.points[face.pointIndexes[i]];

            Vector3D p2;
            if (i + 1 >= face.pointIndexes.size()) p2 = fig.points[face.pointIndexes[0]];
            else p2 = fig.points[face.pointIndexes[i + 1]];
            Line2D line = Line2D(projectPoint(p1, 1.0), projectPoint(p2, 1.0), fig.color);

            lines.push_back(line);
        }


    }

    return lines;
}

Lines2D ProjectAll(const Figures3D& figs) {
    Lines2D lines;

    for (auto fig : figs) {
        Lines2D figLines = projectFig(fig);
        lines.insert(lines.end(), figLines.begin(), figLines.end());
    }

    return lines;
}

img::EasyImage colorRectangle(const ini::Configuration& configuration) {
    int w;
    int h;

    if (!configuration["ImageProperties"]["width"].as_int_if_exists(w)) std::cout << "⛔️|Failed to fetch width" << std::endl;
    if (!configuration["ImageProperties"]["height"].as_int_if_exists(h)) std::cout << "⛔️|Failed to fetch height" << std::endl;

    img::EasyImage img(w, h);
    for (unsigned int i = 0; i < w; i++) {
        for (unsigned int j = 0; j < h; j++) {
            img(i, j).red = i;
            img(i, j).green = j;
            img(i, j).blue = (i + j) % w;
        }
    }

    return img;
}

img::EasyImage blocks(const ini::Configuration& configuration) {
    int w;
    int h;

    if (!configuration["ImageProperties"]["width"].as_int_if_exists(w)) std::cout << "⛔️|Failed to fetch width" << std::endl;
    if (!configuration["ImageProperties"]["height"].as_int_if_exists(h)) std::cout << "⛔️|Failed to fetch height" << std::endl;

    std::vector<double> colorWhite;
    std::vector<double> colorBlack;
    std::vector<double> color;
    std::vector<double> otherColor;
    int nrXBlocks;
    int nrYBlocks;
    bool invertColors;

    if (!configuration["BlockProperties"]["colorWhite"].as_double_tuple_if_exists(colorWhite)) std::cout << "⛔️| Failed to fetch color 'white'" << std::endl;
    if (!configuration["BlockProperties"]["colorBlack"].as_double_tuple_if_exists(colorBlack)) std::cout << "⛔️| Failed to fetch color 'black'" << std::endl;
    if (!configuration["BlockProperties"]["nrXBlocks"].as_int_if_exists(nrXBlocks)) std::cout << "⛔️| Failed to fetch number of blocks on axis 'x'" << std::endl;
    if (!configuration["BlockProperties"]["nrYBlocks"].as_int_if_exists(nrYBlocks)) std::cout << "⛔️| Failed to fetch number of blocks on axis 'y''" << std::endl;
    if (!configuration["BlockProperties"]["invertColors"].as_bool_if_exists(invertColors)) std::cout << "⛔️| Failed to fetch boolean 'invert colors'" << std::endl;

    img::EasyImage img(w, h);
    int wB = w / nrXBlocks;
    int hB = h / nrYBlocks;

    for (int i = 0; i < w; i++) {
        for (int j = 0; j < w; j++) {
            if ((i / wB + j / hB) % 2 == 0) {
                img(i, j).red = colorWhite[0] * 255;
                img(i, j).green = colorWhite[1] * 255;
                img(i, j).blue = colorWhite[2] * 255;
            }
            else {
                img(i, j).red = colorBlack[0] * 255;
                img(i, j).green = colorBlack[1] * 255;
                img(i, j).blue = colorBlack[2] * 255;
            }
        }
    }

    return img;
}

img::EasyImage introLines(const ini::Configuration& configuration) {
    int w;
    int h;

    if (!configuration["ImageProperties"]["width"].as_int_if_exists(w)) std::cout << "⛔️|Failed to fetch width" << std::endl;
    if (!configuration["ImageProperties"]["height"].as_int_if_exists(h)) std::cout << "⛔️|Failed to fetch height" << std::endl;

    std::string figure;
    std::vector<double> backgroundColor;
    std::vector<double> lineColor;
    int nrLines;

    if (!configuration["LineProperties"]["figure"].as_string_if_exists(figure)) std::cout << "⛔️| Failed to fetch" << std::endl;
    if (!configuration["LineProperties"]["backgroundcolor"].as_double_tuple_if_exists(backgroundColor)) std::cout << "⛔️|Failed to fetch width" << std::endl;
    if (!configuration["LineProperties"]["lineColor"].as_double_tuple_if_exists(lineColor)) std::cout << "⛔️|Failed to fetch height" << std::endl;
    if (!configuration["LineProperties"]["nrLines"].as_int_if_exists(nrLines)) std::cout << "⛔️|Failed to fetch height" << std::endl;

    img::Color bg;
    bg.red = backgroundColor[0] * 255;
    bg.green = backgroundColor[1] * 255;
    bg.blue = backgroundColor[2] * 255;

    img::Color line;
    line.red = lineColor[0] * 255;
    line.green = lineColor[1] * 255;
    line.blue = lineColor[2] * 255;

    std::cout << figure << std::endl;

    img::EasyImage img(w, h, bg);
    int hS = h / (nrLines - 1);
    int wS = w / (nrLines - 1);

    if (figure == "QuarterCircle") {
        for (int i = 0; i < w; i += wS) {
            img.draw_line(i, h - 1, 0, i, line);
        }
    }

    if (figure == "Eye") {
        for (int i = 0; i < w; i += wS) {
            img.draw_line(i, h - 1, 0, i, line);
        }

        for (int i = 0; i < w; i += wS) {
            img.draw_line(h - i - 1, 0, h - 1, h - i - 1, line);
        }
    }

    if (figure == "Diamond") {
        w = w / 2;
        h = h / 2;
        hS = h / (nrLines - 1);
        wS = w / (nrLines - 1);

        for (int i = 0; i < w; i += wS) {
            img.draw_line(h - i - 1, h - 1, h - 1, i, line);
        }

        int offsetX = h;
        int offsetY = 0;

        for (int i = 0; i < w; i += wS) {
            img.draw_line(offsetX + i, offsetY + h - 1, offsetX + 0, offsetY + i, line);
        }

        offsetX = h;
        offsetY = h;

        for (int i = 0; i < w; i += wS) {
            img.draw_line(offsetX + i, offsetY + 0, offsetX + 0, offsetY + h - i - 1, line);
        }

        offsetX = 0;

        for (int i = 0; i < w; i += wS) {
            img.draw_line(offsetX + h - i - 1, +offsetY + 0, offsetX + h - 1, offsetY + h - i - 1, line);
        }
    }

    return img;
}

struct DoubleTriplet {
    double  first, second, third;
};


void draw2DLSystem(const LParser::LSystem2D& l_system, Lines2D& lines, const Color color, std::string current = "", int it = 0) {
    int iterations = l_system.get_nr_iterations();
    std::string initiator = l_system.get_initiator();
    std::set<char> alphabet = l_system.get_alphabet();

    if (current == "") current = initiator;

    if (it == iterations) {
        double angle = l_system.get_starting_angle();
        double angleOffset = l_system.get_angle();

        std::stack<DoubleTriplet> stack;
        double x = 0;
        double y = 0;


        for (char c : current) {
            if (c == '+') angle += angleOffset;
            else if (c == '-') angle -= angleOffset;
            else if (c == '(') stack.push(DoubleTriplet{ x, y, angle });
            else if (c == ')') {
                DoubleTriplet saved = stack.top();
                stack.pop();

                x = saved.first;
                y = saved.second;
                angle = saved.third;
            }
            else if (alphabet.find(c) != alphabet.end()) {
                if (l_system.draw(c)) {
                    Point2D p1 = Point2D(x, y);

                    x += std::cos(angle * M_PI / 180);
                    y += std::sin(angle * M_PI / 180);

                    Point2D p2 = Point2D(x, y);

                    Line2D line = Line2D(p1, p2, color);
                    lines.push_back(line);
                }
                else {
                    x += std::cos(angle * M_PI / 180);
                    y += std::sin(angle * M_PI / 180);
                }
            }
            else std::cout << ("⛔️| Invalid character " + std::to_string(c) + " in l-system description") << std::endl;
        }

        return;
    }

    std::string replaced = "";
    for (char c : current) {
        if (alphabet.find(c) == alphabet.end()) replaced += c;
        else {
            replaced += l_system.get_replacement(c);
        }
    }

    draw2DLSystem(l_system, lines, color, replaced, it + 1);
}

img::EasyImage LSystem(const ini::Configuration& configuration) {
    int size;
    std::vector<double> backgroundColorRaw;
    if (!configuration["General"]["size"].as_int_if_exists(size)) std::cout << "⛔️| Failed to fetch size" << std::endl;
    if (!configuration["General"]["backgroundcolor"].as_double_tuple_if_exists(backgroundColorRaw)) std::cout << "⛔️| Failed to fetch background color" << std::endl;

    Color backgroundColor = Color(backgroundColorRaw[0], backgroundColorRaw[1], backgroundColorRaw[2]);

    std::string file;
    std::vector<double> colorRaw;

    if (!configuration["2DLSystem"]["inputfile"].as_string_if_exists(file)) std::cout << "⛔️| Failed to fetch input file" << std::endl;
    if (!configuration["2DLSystem"]["color"].as_double_tuple_if_exists(colorRaw)) std::cout << "⛔️|Failed to fetch color" << std::endl;

    Color color = Color(colorRaw[0], colorRaw[1], colorRaw[2]);
    LParser::LSystem2D l_system;

    std::ifstream input_stream(file);
    input_stream >> l_system;
    input_stream.close();

    Lines2D lines;
    draw2DLSystem(l_system, lines, color);
    img::EasyImage img = draw2DLines(lines, size, backgroundColor);


    return img;
}

struct Vector3DQuadruplet {
    Vector3D first, second, third, fourth;
};

void draw3DLSystem(const LParser::LSystem3D& l_system, Figure& figure, const Color color, std::string current = "", int it = 0) {
    int iterations = l_system.get_nr_iterations();
    std::string initiator = l_system.get_initiator();
    std::set<char> alphabet = l_system.get_alphabet();

    if (current == "") current = initiator;

    if (it == iterations) {
        Vector3D cur = Vector3D::point(0, 0, 0);

        Vector3D h = Vector3D::point(1, 0, 0);
        Vector3D l = Vector3D::point(0, 1, 0);
        Vector3D u = Vector3D::point(0, 0, 1);

        double angleOffset = l_system.get_angle() * M_PI / 180;

        std::stack<Vector3DQuadruplet> stack;

        for (char c : current) {
            if (c == '+') {
                Vector3D newH = h * std::cos(angleOffset) + l * std::sin(angleOffset);
                Vector3D newL = -h * std::sin(angleOffset) + l * std::cos(angleOffset);

                h = newH;
                l = newL;
            }
            else if (c == '-') {
                Vector3D newH = h * std::cos(-angleOffset) + l * std::sin(-angleOffset);
                Vector3D newL = -h * std::sin(-angleOffset) + l * std::cos(-angleOffset);

                h = newH;
                l = newL;
            }
            else if (c == '^') {
                Vector3D newH = h * std::cos(angleOffset) + u * std::sin(angleOffset);
                Vector3D newU = -h * std::sin(angleOffset) + u * std::cos(angleOffset);

                h = newH;
                u = newU;
            }
            else if (c == '&') {
                Vector3D newH = h * std::cos(-angleOffset) + u * std::sin(-angleOffset);
                Vector3D newU = -h * std::sin(-angleOffset) + u * std::cos(-angleOffset);

                h = newH;
                u = newU;
            }
            else if (c == '\\') {
                Vector3D newL = l * std::cos(angleOffset) - u * std::sin(angleOffset);
                Vector3D newU = l * std::sin(angleOffset) + u * std::cos(angleOffset);

                l = newL;
                u = newU;
            }
            else if (c == '/') {
                Vector3D newL = l * std::cos(-angleOffset) - u * std::sin(-angleOffset);
                Vector3D newU = l * std::sin(-angleOffset) + u * std::cos(-angleOffset);

                l = newL;
                u = newU;
            }
            else if (c == '|') {
                h = -h;
                l = -l;
            }
            else if (c == '(') stack.push(Vector3DQuadruplet{ cur, h, l, u });
            else if (c == ')') {
                Vector3DQuadruplet saved = stack.top();
                stack.pop();

                cur = saved.first;
                h = saved.second;
                l = saved.third;
                u = saved.fourth;
            }
            else if (alphabet.find(c) != alphabet.end()) {
                if (l_system.draw(c)) {
                    Vector3D p1 = cur;
                    cur += h;
                    Vector3D p2 = cur;

                    figure.points.push_back(p1);
                    int i = figure.points.size() - 1;
                    figure.points.push_back(p2);

                    figure.faces.push_back(Face({ i, i + 1 }));
                }
                else cur += h;
            }
            else std::cout << ("⛔️| Invalid character " + std::to_string(c) + " in l-system description") << std::endl;
        }

        return;
    }

    std::string replaced = "";
    for (char c : current) {
        if (alphabet.find(c) == alphabet.end()) replaced += c;
        else {
            replaced += l_system.get_replacement(c);
        }
    }

    draw3DLSystem(l_system, figure, color, replaced, it + 1);
}

img::EasyImage wireFrame(const ini::Configuration& c) {
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
        int rotateX;
        int rotateY;
        int rotateZ;
        if (!base["scale"].as_double_if_exists(scale)) std::cout << "⛔️| Failed to fetch scale" << std::endl;
        if (!base["center"].as_double_tuple_if_exists(center)) std::cout << "⛔️| Failed to fetch center" << std::endl;
        if (!base["rotateX"].as_int_if_exists(rotateX)) std::cout << "⛔️| Failed to fetch rotateX" << std::endl;
        if (!base["rotateY"].as_int_if_exists(rotateY)) std::cout << "⛔️| Failed to fetch rotateY" << std::endl;
        if (!base["rotateZ"].as_int_if_exists(rotateZ)) std::cout << "⛔️| Failed to fetch rotateZ" << std::endl;

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
        else if (type == "Cylinder") {
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
        else if (type == "3DLSystem") {
            std::string inputFile;
            if (!base["inputfile"].as_string_if_exists(inputFile)) std::cout << "⛔️| Failed to fetch # points" << std::endl;

            LParser::LSystem3D l_system;
            std::ifstream input_stream(inputFile);
            input_stream >> l_system;
            input_stream.close();

            figure.color = color;

            draw3DLSystem(l_system, figure, color);
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

    Lines2D lines = ProjectAll(figures);
    return draw2DLines(lines, size, Color(0, 0, 0));
}

img::EasyImage generate_image(const ini::Configuration& configuration) {
    std::string t;

    if (!configuration["General"]["type"].as_string_if_exists(t)) std::cout << "⛔️| Failed to fetch type" << std::endl;

    std::cout << "🥱| Generating image of type '" + t + "'" << std::endl;

    img::EasyImage result;
    if (t == "IntroColorRectangle") result = colorRectangle(configuration);
    if (t == "IntroBlocks") result = blocks(configuration);
    if (t == "IntroLines") result = introLines(configuration);
    if (t == "2DLSystem") result = LSystem(configuration);
    if (t == "Wireframe") result = wireFrame(configuration);

    std::cout << "✅| Image generated" << std::endl;

    return result;
}

int main(int argc, char const* argv[]) {
    int retVal = 0;
    try {
        std::vector<std::string> args =
            std::vector<std::string>(argv + 1, argv + argc);
        if (args.empty()) {
            std::ifstream fileIn("filelist");
            std::string filelistName;
            while (std::getline(fileIn, filelistName)) {
                args.push_back(filelistName);
            }
        }
        for (std::string fileName : args) {
            ini::Configuration conf;
            try {
                std::ifstream fin(fileName);
                fin >> conf;
                fin.close();
            }
            catch (ini::ParseException& ex) {
                std::cerr << "Error parsing file: " << fileName << ": " << ex.what()
                    << std::endl;
                retVal = 1;
                continue;
            }

            img::EasyImage image = generate_image(conf);
            if (image.get_height() > 0 && image.get_width() > 0) {
                std::string::size_type pos = fileName.rfind('.');
                if (pos == std::string::npos) {
                    // filename does not contain a '.' --> append a '.bmp' suffix
                    fileName += ".bmp";
                }
                else {
                    fileName = fileName.substr(0, pos) + ".bmp";
                }
                try {
                    std::ofstream f_out(fileName.c_str(), std::ios::trunc |
                        std::ios::out |
                        std::ios::binary);
                    f_out << image;
                }
                catch (std::exception& ex) {
                    std::cerr << "Failed to write image to file: " << ex.what()
                        << std::endl;
                    retVal = 1;
                }
            }
            else {
                std::cout << "Could not generate image for " << fileName << std::endl;
            }
        }
    }
    catch (const std::bad_alloc& exception) {
        // When you run out of memory this exception is thrown. When this happens
        // the return value of the program MUST be '100'. Basically this return
        // value tells our automated test scripts to run your engine on a pc with
        // more memory.
        //(Unless of course you are already consuming the maximum allowed amount of
        // memory)
        // If your engine does NOT adhere to this requirement you risk losing points
        // because then our scripts will mark the test as failed while in reality it
        // just needed a bit more memory
        std::cerr << "Error: insufficient memory" << std::endl;
        retVal = 100;
    }
    return retVal;
}
