#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_LIGHTEDZBUFFERING_CC_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_LIGHTEDZBUFFERING_CC_

#include "LightedZBuffering.h"
#include "Universal.h"
#include "../util/Figure.h"
#include "../util/generators/Transformations.h"

img::EasyImage lightedZBuffering(const ini::Configuration& c) {
    Figures3D figures = parseFigures(c);
    Details details = parseGeneralDetails(c);
    Lights3D lights = parseLights(c);

    Matrix eyePointTransMatrix = transformations::eyePointTrans(details.eye);
    applyTransformationAll(figures, eyePointTransMatrix);

    Lines2D lines = projectAll(figures);
    return draw2DLines(lines, details.size, details.backgroundColor, true);
}

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_LIGHTEDZBUFFERING_CC_