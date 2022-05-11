#include "ZBufferTriangle.h"

#include "../util/generators/PlatonicSolids.h"
#include "../util/generators/Transformations.h"
#include "../lib/l_parser/l_parser.h"
#include "Universal.h"
#include <fstream>
#include <math.h>
#include <cmath>


img::EasyImage zBufferTriangle(const ini::Configuration& c) {
    Figures3D figures = parseFigures(c);
    Details details = parseGeneralDetails(c);
    Lights3D lights;

    return drawFigures(figures, details.eye, details.size, details.backgroundColor, lights, false);
}