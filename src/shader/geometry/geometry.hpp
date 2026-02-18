#ifndef CRIDGEON_SHADER_GEOMETRY_HPP
#define CRIDGEON_SHADER_GEOMETRY_HPP

#include "circle_filled.hpp"

namespace Render {
    inline void destroyGeometryShaders() {
        _destroyCircleFilled();
    }
}

#endif // CRIDGEON_SHADER_GEOMETRY_HPP