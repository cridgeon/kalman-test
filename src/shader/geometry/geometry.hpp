#ifndef CRIDGEON_SHADER_GEOMETRY_HPP
#define CRIDGEON_SHADER_GEOMETRY_HPP

#include "circle.hpp"
#include "circle_filled.hpp"
#include "polygon_filled.hpp"
#include "lines.hpp"
#include "line.hpp"
#include "polygon.hpp"

namespace Render {
    inline void destroyGeometryShaders() {
        _destroyCircle();
        _destroyCircleFilled();
        _destroyPolygonFilled();
        _destroyLines();
    }
}

#endif // CRIDGEON_SHADER_GEOMETRY_HPP