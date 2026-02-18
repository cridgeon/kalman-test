#ifndef CRIDGEON_SHADER_CIRCLE_FILLED_HPP
#define CRIDGEON_SHADER_CIRCLE_FILLED_HPP

#include "rendering_system.hpp"
#include "../shader.hpp"
#include "../utility.hpp"

namespace Render {

    static Shader circleFilledShader;

    inline void circleFilled(float x, float y, float radius, float r, float g, float b, float a) {
        if (!circleFilledShader.isValid()) {
            circleFilledShader.loadFromFile("resources/shaders/geometry/default.vert", "resources/shaders/geometry/circle_filled.frag");
            if (!circleFilledShader.isValid()) {
                throw std::runtime_error("Failed to load circle_filled shader");
            }
        }
        circleFilledShader.use();
        float w = RenderingSystem::getInstance().getWindowWidth();
        float h = RenderingSystem::getInstance().getWindowHeight();
        glUniform2f(circleFilledShader.getUniformLocation("resolution"), w, h);
        glUniform2f(circleFilledShader.getUniformLocation("position"), x, y);
        glUniform1f(circleFilledShader.getUniformLocation("radius"), radius);
        glUniform4f(circleFilledShader.getUniformLocation("color"), r, g, b, a);

        ShaderUtility::drawFullScreenQuad();
    }

    inline void _destroyCircleFilled() {
        circleFilledShader.destroy();
    }
}

#endif // CRIDGEON_SHADER_CIRCLE_FILLED_HPP