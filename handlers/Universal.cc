#include "Universal.h"

#include <cmath>
#include <list>

#include "../util/Line2D.h"
#include "../util/Figure.h"

void applyTransformation(Figure& fig, const Matrix& m) {
    for (auto& p : fig.points) p *= m;
}

void applyTransformationAll(Figures3D& figs, const Matrix& m) {
    for (auto& f : figs) applyTransformation(f, m);
}

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

ImageDetails getImageDetails(const Lines2D &lines, const double size) {
    // TODO: handle edge case
    Line2D first = lines.front();

    double xMin = first.p1.x;
    double xMax = first.p1.x;

    double yMin = first.p1.y;
    double yMax = first.p1.y;

    // Determine min and max
    for (const Line2D& line : lines) {
        if (line.p1.x < xMin) xMin = line.p1.x;
        if (line.p2.x < xMin) xMin = line.p2.x;
        if (line.p1.x > xMax) xMax = line.p1.x;
        if (line.p2.x > xMax) xMax = line.p2.x;

        if (line.p1.y < yMin) yMin = line.p1.y;
        if (line.p2.y < yMin) yMin = line.p2.y;
        if (line.p1.y > yMax) yMax = line.p1.y;
        if (line.p2.y > yMax) yMax = line.p2.y;
    }

    // Compute variables needed for next step
    double xRange = std::abs(xMax - xMin);
    double yRange = std::abs(yMax - yMin);

    double imageX = size * xRange / std::max(xRange, yRange);
    double imageY = size * yRange / std::max(xRange, yRange);

    return ImageDetails{ 
        .imageX = imageX, 
        .imageY = imageY, 
        .xRange = xRange, 
        .yRange = yRange, 
        .xMin = xMin, 
        .xMax = xMax, 
        .yMin = yMin, 
        .yMax = yMax,
    };
}

img::EasyImage draw2DLines(const Lines2D& lines, const int size, Color background, bool zBuffer) {
    ImageDetails details = getImageDetails(lines, (double) size);

    double d = 0.95 * details.imageX / details.xRange;
    double dcX = d * (details.xMin + details.xMax) / 2;
    double dcY = d * (details.yMin + details.yMax) / 2;
    double dX = details.imageX / 2 - dcX;
    double dY = details.imageY / 2 - dcY;

    // Create image
    img::EasyImage img(std::lround(details.imageX), std::lround(details.imageY), background.toNative());

    ZBuffer z = ZBuffer(std::lround(details.imageX), std::lround(details.imageY));

    // Re-position points
    for (Line2D line : lines) {
        line.p1.x *= d;
        line.p1.y *= d;

        line.p1.x += dX;
        line.p1.y += dY;

        line.p1.x = std::lround(line.p1.x);
        line.p1.y = std::lround(line.p1.y);

        line.p2.x *= d;
        line.p2.y *= d;

        line.p2.x += dX;
        line.p2.y += dY;

        line.p2.x = std::lround(line.p2.x);
        line.p2.y = std::lround(line.p2.y);

        if (zBuffer) {
            img.draw_zbuf_line(z, line.p1.x, line.p1.y, line.z1, line.p2.x, line.p2.y, line.z2, line.color.toNative());
        } else img.draw_line(line.p1.x, line.p1.y, line.p2.x, line.p2.y, line.color.toNative());
    }

    return img;
}

Point2D projectPoint(const Vector3D& point, const double d) {
    double x = d * point.x / -point.z;
    double y = d * point.y / -point.z;

    return Point2D(x, y);
}

Lines2D projectFig(const Figure& fig) {
    Lines2D lines;

    for (auto face : fig.faces) {
        for (unsigned int i = 0; i < face.pointIndexes.size(); i++) {
            Vector3D p1 = fig.points[face.pointIndexes[i]];

            Vector3D p2;
            if (i + 1 >= face.pointIndexes.size()) p2 = fig.points[face.pointIndexes[0]];
            else p2 = fig.points[face.pointIndexes[i + 1]];
            Line2D line = Line2D(projectPoint(p1, 1.0), projectPoint(p2, 1.0), p1.z, p2.z, fig.color);

            lines.push_back(line);
        }


    }

    return lines;
}

Lines2D ProjectAll(const Figures3D& figs) {
    Lines2D lines;

    for (auto fig : figs) {
        Lines2D figLines = projectFig(fig);
        lines.insert(lines.end(), figLines.begin(), figLines.end());
    }

    return lines;
}