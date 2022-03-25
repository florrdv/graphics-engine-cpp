#include "ZBuffer.h"

#include <limits>

ZBuffer::ZBuffer(const int w, const int h) { 
    for (int i = 0; i < w; i++) { 
        for (int j = 0; j < h; j++) { 
            (*this)[i][j] = std::numeric_limits<double>::infinity();
        }
    }
}