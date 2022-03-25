#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_UNIVERSAL_H_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_UNIVERSAL_H_

#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_UNIVERSAL_CC_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_UNIVERSAL_CC_

#include <list>

#include "../util/Figure.h"
#include "../util/Line2D.h"
#include "../easy_image.h"

using Lines2D = std::list<Line2D>;
using Figures3D = std::list<Figure>;

void applyTransformation(Figure& fig, const Matrix& m);

void applyTransformationAll(Figures3D& figs, const Matrix& m);

img::EasyImage draw2DLines(const Lines2D& lines, const int size, Color background, bool zBuffer = false);

Point2D projectPoint(const Vector3D& point, const double d);

Lines2D projectFig(const Figure& fig);

Lines2D ProjectAll(const Figures3D& figs);

Lines2D projectFig(const Figure& fig);

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_UNIVERSAL_CC_

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_UNIVERSAL_H_