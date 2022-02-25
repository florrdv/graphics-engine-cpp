#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "easy_image.h"
#include "ini_configuration.h"

img::EasyImage colorRectangle(int w, int h) {
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

img::EasyImage blocks(const ini::Configuration& configuration, int w, int h) {
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

img::EasyImage generate_image(const ini::Configuration& configuration) {
    std::string t;
    int w;
    int h;

    if (!configuration["General"]["type"].as_string_if_exists(t)) std::cout << "⛔️| Failed to fetch" << std::endl;
    if (!configuration["ImageProperties"]["width"].as_int_if_exists(w)) std::cout << "⛔️|Failed to fetch width" << std::endl;
    if (!configuration["ImageProperties"]["height"].as_int_if_exists(h)) std::cout << "⛔️|Failed to fetch height" << std::endl;

    std::cout << "🥱| Generating image of type '" + t + "'" << std::endl;
    std::cout << "🥱| Using dimensions " + std::to_string(w) + "x" + std::to_string(h) << std::endl;

    img::EasyImage result;
    if (t == "IntroColorRectangle") result = colorRectangle(w, h);
    if (t == "IntroBlocks") result = blocks(configuration, w, h);

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
