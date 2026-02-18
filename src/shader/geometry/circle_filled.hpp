#ifndef CRIDGEON_SHADER_CIRCLE_FILLED_HPP
#define CRIDGEON_SHADER_CIRCLE_FILLED_HPP

#include "rendering_system.hpp"
#include "../shader.hpp"
#include "../utility.hpp"

namespace Render {

    static Shader s;

    inline void circleFilled(float x, float y, float radius, float r, float g, float b, float a) {
        if (!s.isValid()) {
            s.loadFromFile("resources/shaders/geometry/default.vert", "resources/shaders/geometry/circle_filled.frag");
            if (!s.isValid()) {
                throw std::runtime_error("Failed to load circle_filled shader");
            }
        }
        s.use();
        float w = RenderingSystem::getInstance().getWindowWidth();
        float h = RenderingSystem::getInstance().getWindowHeight();
        glUniform2f(s.getUniformLocation("resolution"), w, h);
        glUniform2f(s.getUniformLocation("position"), x, y);
        glUniform1f(s.getUniformLocation("radius"), radius);
        glUniform4f(s.getUniformLocation("color"), r, g, b, a);

        ShaderUtility::drawFullScreenQuad();
    }

    inline void _destroyCircleFilled() {
        s.destroy();
    }
}

#endif // CRIDGEON_SHADER_CIRCLE_FILLED_HPP