#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_2DLSYSTEM_H_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_2DLSYSTEM_H_

#include "../lib/l_parser/l_parser.h"
#include "../ini_configuration.h"
#include "Universal.h"

void draw2DLSystem(const LParser::LSystem2D& l_system, Lines2D& lines, const Color color, std::string current = "", int it = 0);

img::EasyImage LSystem(const ini::Configuration& configuration);

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_2DLSYSTEM_H_