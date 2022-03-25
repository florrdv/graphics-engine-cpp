#include "2DLsystem.h"

#include <string>
#include <stack>
#include <fstream>
#include <cmath>

#include "../lib/l_parser/l_parser.h"
#include "../ini_configuration.h"
#include "Universal.h"

struct DoubleTriplet {
    double  first, second, third;
};


void draw2DLSystem(const LParser::LSystem2D& l_system, Lines2D& lines, const Color color, std::string current, int it) {
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