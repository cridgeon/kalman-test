#ifndef CRIDGEON_SHADER_LINE_HPP
#define CRIDGEON_SHADER_LINE_HPP

#include "lines.hpp"
#include <vector>

namespace Render {

    inline void line(float x1, float y1, float x2, float y2, float r, float g, float b, float a) {
        std::vector<float> vertices = {x1, y1, x2, y2};
        lines(vertices, r, g, b, a);
    }

}

#endif // CRIDGEON_SHADER_LINE_HPP
