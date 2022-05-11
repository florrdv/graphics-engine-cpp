#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_UNIVERSAL_H_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_UNIVERSAL_H_

#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_UNIVERSAL_CC_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_UNIVERSAL_CC_

#include <list>

#include "../util/Figure.h"
#include "../util/Line2D.h"
#include "../easy_image.h"
#include "../util/Light.h"
#include "../ini_configuration.h"

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

struct Details {
    int size;
    Vector3D eye;
    Color backgroundColor;
};

void applyTransformation(Figure& fig, const Matrix& m);

void applyTransformationAll(Figures3D& figs, const Matrix& m);

img::EasyImage draw2DLines(const Lines2D& lines, const int size, Color background, bool zBuffer = false);

Point2D projectPoint(const Vector3D& point, const double d);

Lines2D projectFig(const Figure& fig);

Lines2D projectAll(const Figures3D& figs);

Lines2D projectFig(const Figure& fig);

ImageDetails getImageDetails(const Lines2D &lines, const double size);

void draw_zbuf_triag(ZBuffer &z, img::EasyImage &img, Matrix &eyeM,
                    Vector3D const& A, Vector3D const& B, Vector3D const& C, 
                    double d, double dx, double dy, 
                    Color ambientReflection, Color diffuseReflection, Color specularReflection, double reflectionCoeff,
                    Lights3D& lights);

Figures3D parseFigures(const ini::Configuration& c);
Details parseGeneralDetails(const ini::Configuration& c);
Lights3D parseLights(const ini::Configuration& c);
void drawFigure(img::EasyImage &img, Vector3D& eye, ZBuffer &z, Figure &f, double size, double d, double dX, double dY, Color &background, Lights3D &lights);
img::EasyImage drawFigures(Figures3D &figures, Matrix &eyeM, double size, Color &background, Lights3D &lights);

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_UNIVERSAL_CC_

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_HANDLERS_UNIVERSAL_H_