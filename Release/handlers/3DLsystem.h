#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_3DLSYSTEM_H_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_3DLSYSTEM_H_

#include "../lib/l_parser/l_parser.h"
#include "Universal.h"

void draw3DLSystem(const LParser::LSystem3D& l_system, Figure& figure, const Color color, std::string current = "", int it = 0);

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_3DLSYSTEM_H_