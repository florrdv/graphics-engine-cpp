#include "WireFrame.h"

#include <fstream>
#include <math.h>

#include "Universal.h"
#include "3DLsystem.h"
#include "Fractals.h"
#include "../ini_configuration.h"
#include "../util/generators/PlatonicSolids.h"
#include "../util/generators/Transformations.h"

img::EasyImage wireFrame(const ini::Configuration& c, bool zBuffer) {
    Figures3D figures = parseFigures(c);
    Details details = parseGeneralDetails(c);

    Matrix eyePointTransMatrix = transformations::eyePointTrans(details.eye);
    applyTransformationAll(figures, eyePointTransMatrix);

    Lines2D lines = projectAll(figures);
    return draw2DLines(lines, details.size, details.backgroundColor, zBuffer);
}