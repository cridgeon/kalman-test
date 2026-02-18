#ifndef CRIDGEON_SHADER_ALL_HPP
#define CRIDGEON_SHADER_ALL_HPP

#include "geometry/geometry.hpp"

namespace Render {
    inline void destroyAllShaders() {
        destroyGeometryShaders();
    }
}

#endif // CRIDGEON_SHADER_ALL_HPP