#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_ZBUFFERINGTRIANGLES_H_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_ZBUFFERINGTRIANGLES_H_

#include "../easy_image.h"
#include "../ini_configuration.h"
#include "Universal.h"

img::EasyImage zBufferTriangle(const ini::Configuration& c);
img::EasyImage drawFigures(Figures3D &figures, double size, Color &background, bool shadows);

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_ZBUFFERINGTRIANGLES_H_