#ifndef CRIDGEON_SHADER_POLYGON_HPP
#define CRIDGEON_SHADER_POLYGON_HPP

#include "lines.hpp"
#include <vector>

namespace Render {

    inline void polygon(const std::vector<float>& vertices, float r, float g, float b, float a) {
        if (vertices.size() < 6) return; // Need at least 3 vertices (6 floats) for a polygon
        
        // Create line segments that form a closed loop
        std::vector<float> lineVertices;
        lineVertices.reserve(vertices.size() * 2); // +2 for closing the loop
        
        lineVertices.push_back(vertices[0]);
        lineVertices.push_back(vertices[1]);
        
        // Add all vertices as line segments
        for (size_t i = 2; i < vertices.size(); i += 2) {
            lineVertices.push_back(vertices[i]);
            lineVertices.push_back(vertices[i + 1]);
            lineVertices.push_back(vertices[i]);
            lineVertices.push_back(vertices[i + 1]);
        }
        
        // Close the polygon by connecting last vertex back to first
        lineVertices.push_back(vertices[0]);
        lineVertices.push_back(vertices[1]);
        
        // Use GL_LINE_STRIP by calling lines with connected vertices
        lines(lineVertices, r, g, b, a);
    }

}

#endif // CRIDGEON_SHADER_POLYGON_HPP
