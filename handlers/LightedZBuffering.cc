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

    return drawFigures(figures, eyePointTransMatrix, details.size, details.backgroundColor, lights);
}

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_LIGHTEDZBUFFERING_CC_