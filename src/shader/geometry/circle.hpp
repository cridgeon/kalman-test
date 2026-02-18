#ifndef CRIDGEON_SHADER_CIRCLE_HPP
#define CRIDGEON_SHADER_CIRCLE_HPP

#include "rendering_system.hpp"
#include "../shader.hpp"
#include "../utility.hpp"

namespace Render {

    static Shader circleShader;

    inline void circle(float x, float y, float radius, float r, float g, float b, float a) {
        if (!circleShader.isValid()) {
            circleShader.loadFromFile("resources/shaders/geometry/default.vert", "resources/shaders/geometry/circle.frag");
            if (!circleShader.isValid()) {
                throw std::runtime_error("Failed to load circle shader");
            }
        }
        circleShader.use();
        float w = RenderingSystem::getInstance().getWindowWidth();
        float h = RenderingSystem::getInstance().getWindowHeight();
        glUniform2f(circleShader.getUniformLocation("resolution"), w, h);
        glUniform2f(circleShader.getUniformLocation("position"), x, y);
        glUniform1f(circleShader.getUniformLocation("radius"), radius);
        glUniform4f(circleShader.getUniformLocation("color"), r, g, b, a);

        ShaderUtility::drawFullScreenQuad();
    }

    inline void _destroyCircle() {
        circleShader.destroy();
    }
}

#endif // CRIDGEON_SHADER_CIRCLE_FILLED_HPP