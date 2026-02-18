#version 130

in vec2 TexCoords;
out vec4 color;

uniform sampler2D screenTexture;

void main() {
    // Simple passthrough - no effect
    color = texture2D(screenTexture, TexCoords);
}