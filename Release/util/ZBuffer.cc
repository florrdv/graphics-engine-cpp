#include "ZBuffer.h"

#include <limits>

ZBuffer::ZBuffer(const int w, const int h) {
    for (int i = 0; i < w; i++) { 
        std::vector<double> line;
        for (int j = 0; j < h; j++) { 
            line.push_back(std::numeric_limits<double>::infinity());
        }
        (*this).push_back(line);
    }
}