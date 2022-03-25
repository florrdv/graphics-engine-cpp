img::EasyImage colorRectangle(const ini::Configuration& configuration) {
    int w;
    int h;

    if (!configuration["ImageProperties"]["width"].as_int_if_exists(w)) std::cout << "⛔️|Failed to fetch width" << std::endl;
    if (!configuration["ImageProperties"]["height"].as_int_if_exists(h)) std::cout << "⛔️|Failed to fetch height" << std::endl;

    img::EasyImage img(w, h);
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
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
    // int hS = h / (nrLines - 1);
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
        // hS = h / (nrLines - 1);
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