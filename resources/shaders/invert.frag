#version 130

in vec2 TexCoords;
out vec4 color;

uniform sampler2D screenTexture;

void main() {
    vec3 texColor = texture2D(screenTexture, TexCoords).rgb;
    // Invert the colors
    color = vec4(1.0 - texColor, 1.0);
}