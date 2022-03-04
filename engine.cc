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
#include "lib/l_parser/l_parser.h"


using Lines2D = std::list<Line2D>;

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


void drawLSystem(const LParser::LSystem2D& l_system, Lines2D& lines, const Color color, std::string current = "", int it = 0) {
    int iterations = l_system.get_nr_iterations();
    std::string initiator = l_system.get_initiator();
    std::set<char> alphabet = l_system.get_alphabet();

    if (current == "") current = initiator;

    if (it == iterations) {
        double angle = l_system.get_starting_angle();
        double angleOffset = l_system.get_angle();

        double x = 0;
        double y = 0;

        std::stack<std::vector<double>> stack;

        for (char c : current) {
            if (c == '+') angle += angleOffset;
            else if (c == '-') angle -= angleOffset;
            else if (c == '(') stack.push({x, y, angle});
            else if (c == ')') {
                std::vector<double> saved = stack.top();
                stack.pop();
                
                x = saved[0];
                y = saved[1];
                angle = saved[2];
            } else if (alphabet.find(c) != alphabet.end()) {
                if (l_system.draw(c)) {
                    Point2D p1 = Point2D(x, y);

                    x += std::cos(angle * M_PI / 180);
                    y += std::sin(angle * M_PI / 180);

                    Point2D p2 = Point2D(x, y);

                    Line2D line = Line2D(p1, p2, color);
                    lines.push_back(line);
                }
            } else std::cout << ("⛔️| Invalid character " + std::to_string(c) + " in l-system description") << std::endl;
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

    drawLSystem(l_system, lines, color, replaced, it + 1);
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
    drawLSystem(l_system, lines, color);
    img::EasyImage img = draw2DLines(lines, size, backgroundColor);


    return img;
}

img::EasyImage generate_image(const ini::Configuration& configuration) {
    std::string t;

    if (!configuration["General"]["type"].as_string_if_exists(t)) std::cout << "⛔️| Failed to fetch" << std::endl;

    std::cout << "🥱| Generating image of type '" + t + "'" << std::endl;

    img::EasyImage result;
    if (t == "IntroColorRectangle") result = colorRectangle(configuration);
    if (t == "IntroBlocks") result = blocks(configuration);
    if (t == "IntroLines") result = introLines(configuration);
    if (t == "2DLSystem") result = LSystem(configuration);

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
