#ifndef CRIDGEON_SHADER_UTILITY_HPP
#define CRIDGEON_SHADER_UTILITY_HPP

#include <glad/gl.h>

namespace ShaderUtility {

    static bool fullScreenQuadInitialized_ = false;
    static unsigned int fullScreenQuadVAO_ = UINT_MAX;
    static unsigned int fullScreenQuadVBO_ = UINT_MAX;
    inline void drawFullScreenQuad() {
        if (!fullScreenQuadInitialized_) {
            // Fullscreen quad vertices (2 triangles)
            float vertices[] = {
                // First triangle
                -1.0f,  -1.0f,
                1.0f,  -1.0f,
                1.0f,  1.0f,
                // Second triangle  
                -1.0f,  -1.0f,
                1.0f,  1.0f,
                -1.0f,  1.0f
            };
            
            glGenVertexArrays(1, &fullScreenQuadVAO_);
            glGenBuffers(1, &fullScreenQuadVBO_);
            
            glBindVertexArray(fullScreenQuadVAO_);
            glBindBuffer(GL_ARRAY_BUFFER, fullScreenQuadVBO_);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
            
            // Position attribute
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);
            
            glBindVertexArray(0);
            fullScreenQuadInitialized_ = true;
        }

        glBindVertexArray(fullScreenQuadVAO_);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glBindVertexArray(0);
        
        return;
    }

}

#endif // CRIDGEON_SHADER_UTILITY_HPP