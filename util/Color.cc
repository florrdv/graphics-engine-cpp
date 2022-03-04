#include "Color.h"

img::Color Color::toNative() {
    img::Color color;
    color.red = 255 * red;
    color.green = 255 * green;
    color.blue  = 255 * blue;

    return color;
}