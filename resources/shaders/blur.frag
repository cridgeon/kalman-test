#version 130

in vec2 TexCoords;
out vec4 color;

uniform sampler2D screenTexture;
uniform vec2 resolution;

void main() {
    vec2 texelSize = 1.0 / resolution;
    vec3 result = vec3(0.0);
    
    // Simple 3x3 box blur
    for(int x = -1; x <= 1; ++x) {
        for(int y = -1; y <= 1; ++y) {
            vec2 offset = vec2(float(x), float(y)) * texelSize;
            result += texture2D(screenTexture, TexCoords + offset).rgb;
        }
    }
    
    result /= 9.0;  // Average of 9 samples
    color = vec4(result, 1.0);
}