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

struct ImageDetails {
    double imageX;
    double imageY;

    double xRange;
    double yRange;

    double xMin;
    double xMax;

    double yMin;
    double yMax;
};

void applyTransformation(Figure& fig, const Matrix& m);

void applyTransformationAll(Figures3D& figs, const Matrix& m);

img::EasyImage draw2DLines(const Lines2D& lines, const int size, Color background, bool zBuffer = false);

Point2D projectPoint(const Vector3D& point, const double d);

Lines2D projectFig(const Figure& fig);

Lines2D projectAll(const Figures3D& figs);

Lines2D projectFig(const Figure& fig);

ImageDetails getImageDetails(const Lines2D &lines, const double size);

void draw_zbuf_triag(ZBuffer &z, img::EasyImage &img, 
                    Vector3D const& A, Vector3D const& B, Vector3D const& C, 
                    double d, double dx, double dy, 
                    Color color);

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_UNIVERSAL_CC_

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_UNIVERSAL_H_