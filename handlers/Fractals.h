#include "Universal.h"
#include "../ini_configuration.h"

void generateFractal(Figure& fig, Figures3D& fractal, const int nr_iterations, const double scale);
img::EasyImage drawFractal(const ini::Configuration& c);