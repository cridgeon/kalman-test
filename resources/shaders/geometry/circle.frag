#version 130
in vec2 pixelCoord;
out vec4 fragColor;

uniform vec2 position;
uniform float radius;
uniform vec4 color;

void main() {
    // Calculate distance from pixel to circle center
    float dist = length(pixelCoord - position);
    
    // Create filled circle with anti-aliasing
    float edge_softness = 1.0; // Softness of circle edge for anti-aliasing
    if (dist > radius + edge_softness) discard;
    if (dist < radius - edge_softness) discard;
    fragColor = color;
}