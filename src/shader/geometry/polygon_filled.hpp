#ifndef CRIDGEON_SHADER_POLYGON_FILLED_HPP
#define CRIDGEON_SHADER_POLYGON_FILLED_HPP

#include "rendering_system.hpp"
#include "../shader.hpp"
#include "../utility.hpp"
#include <vector>
#include <cmath>

namespace Render {

    static Shader polygonFilledShader;
    static bool polygonVAOInitialized = false;
    static unsigned int polygonVAO = UINT_MAX;
    static unsigned int polygonVBO = UINT_MAX;

    // Helper function to calculate cross product (for determining triangle orientation)
    inline float cross2D(float x1, float y1, float x2, float y2) {
        return x1 * y2 - x2 * y1;
    }

    // Check if point P is inside triangle ABC
    inline bool isPointInTriangle(float px, float py, 
                                   float ax, float ay, 
                                   float bx, float by, 
                                   float cx, float cy) {
        float v0x = cx - ax, v0y = cy - ay;
        float v1x = bx - ax, v1y = by - ay;
        float v2x = px - ax, v2y = py - ay;

        float dot00 = v0x * v0x + v0y * v0y;
        float dot01 = v0x * v1x + v0y * v1y;
        float dot02 = v0x * v2x + v0y * v2y;
        float dot11 = v1x * v1x + v1y * v1y;
        float dot12 = v1x * v2x + v1y * v2y;

        float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
        float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

        return (u >= 0) && (v >= 0) && (u + v < 1);
    }

    // Check if ear at index i is valid (no other vertices inside)
    inline bool isEar(const std::vector<float>& vertices, const std::vector<int>& indices, int i) {
        int n = indices.size();
        int prev = (i - 1 + n) % n;
        int next = (i + 1) % n;

        int i0 = indices[prev];
        int i1 = indices[i];
        int i2 = indices[next];

        float x0 = vertices[i0 * 2], y0 = vertices[i0 * 2 + 1];
        float x1 = vertices[i1 * 2], y1 = vertices[i1 * 2 + 1];
        float x2 = vertices[i2 * 2], y2 = vertices[i2 * 2 + 1];

        // Check if triangle is counter-clockwise (convex at this vertex)
        float cross = cross2D(x1 - x0, y1 - y0, x2 - x1, y2 - y1);
        if (cross <= 0) return false; // Reflex vertex

        // Check if any other vertex is inside this triangle
        for (int j = 0; j < n; j++) {
            if (j == prev || j == i || j == next) continue;
            int idx = indices[j];
            float px = vertices[idx * 2];
            float py = vertices[idx * 2 + 1];
            if (isPointInTriangle(px, py, x0, y0, x1, y1, x2, y2)) {
                return false;
            }
        }

        return true;
    }

    // Triangulate polygon using ear clipping algorithm
    inline std::vector<float> triangulatePolygon(const std::vector<float>& vertices) {
        int n = vertices.size() / 2;
        if (n < 3) return {};

        std::vector<int> indices;
        for (int i = 0; i < n; i++) {
            indices.push_back(i);
        }

        std::vector<float> triangles;

        while (indices.size() > 3) {
            bool earFound = false;

            for (int i = 0; i < indices.size(); i++) {
                if (isEar(vertices, indices, i)) {
                    int prev = (i - 1 + indices.size()) % indices.size();
                    int next = (i + 1) % indices.size();

                    // Add triangle
                    int i0 = indices[prev];
                    int i1 = indices[i];
                    int i2 = indices[next];

                    triangles.push_back(vertices[i0 * 2]);
                    triangles.push_back(vertices[i0 * 2 + 1]);
                    triangles.push_back(vertices[i1 * 2]);
                    triangles.push_back(vertices[i1 * 2 + 1]);
                    triangles.push_back(vertices[i2 * 2]);
                    triangles.push_back(vertices[i2 * 2 + 1]);

                    // Remove the ear
                    indices.erase(indices.begin() + i);
                    earFound = true;
                    break;
                }
            }

            if (!earFound) {
                // Fallback: just emit remaining triangle if algorithm fails
                break;
            }
        }

        // Add the last triangle
        if (indices.size() == 3) {
            for (int i = 0; i < 3; i++) {
                int idx = indices[i];
                triangles.push_back(vertices[idx * 2]);
                triangles.push_back(vertices[idx * 2 + 1]);
            }
        }

        return triangles;
    }

    inline void polygonFilled(const std::vector<float>& vertices, float r, float g, float b, float a) {
        if (vertices.size() < 6) return; // Need at least 3 vertices (6 floats)

        // Triangulate the polygon
        std::vector<float> triangles = triangulatePolygon(vertices);
        if (triangles.empty()) return;

        // Convert normalized coordinates to screen coordinates
        float w = RenderingSystem::getInstance().getWindowWidth();
        float h = RenderingSystem::getInstance().getWindowHeight();
        
        std::vector<float> screenCoords;
        screenCoords.reserve(triangles.size());
        
        for (size_t i = 0; i < triangles.size(); i += 2) {
            // Convert from pixel coordinates to normalized device coordinates [-1, 1]
            float x = (triangles[i] / w) * 2.0f - 1.0f;
            float y = (triangles[i + 1] / h) * 2.0f - 1.0f;
            screenCoords.push_back(x);
            screenCoords.push_back(y);
        }

        // Load shader if not already loaded
        if (!polygonFilledShader.isValid()) {
            polygonFilledShader.loadFromFile("resources/shaders/geometry/default.vert", "resources/shaders/geometry/color.frag");
            if (!polygonFilledShader.isValid()) {
                throw std::runtime_error("Failed to load polygon_filled shader");
            }
        }

        // Initialize VAO/VBO if needed
        if (!polygonVAOInitialized) {
            glGenVertexArrays(1, &polygonVAO);
            glGenBuffers(1, &polygonVBO);
            polygonVAOInitialized = true;
        }

        polygonFilledShader.use();
        glUniform2f(polygonFilledShader.getUniformLocation("resolution"), w, h);
        glUniform4f(polygonFilledShader.getUniformLocation("color"), r, g, b, a);

        // Upload triangle data
        glBindVertexArray(polygonVAO);
        glBindBuffer(GL_ARRAY_BUFFER, polygonVBO);
        glBufferData(GL_ARRAY_BUFFER, screenCoords.size() * sizeof(float), screenCoords.data(), GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glDrawArrays(GL_TRIANGLES, 0, screenCoords.size() / 2);
        
        glBindVertexArray(0);
    }

    inline void _destroyPolygonFilled() {
        polygonFilledShader.destroy();
        if (polygonVAOInitialized) {
            glDeleteVertexArrays(1, &polygonVAO);
            glDeleteBuffers(1, &polygonVBO);
            polygonVAOInitialized = false;
        }
    }
}

#endif // CRIDGEON_SHADER_POLYGON_FILLED_HPP
