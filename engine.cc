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

Lines2D projectFig(const Figure& fig) {
    Lines2D lines;

    for (auto face : fig.faces) {
        for (unsigned int i = 0; i < face.pointIndexes.size(); i++) {
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
