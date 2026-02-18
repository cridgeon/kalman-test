#version 130

in vec2 TexCoords;
out vec4 color;

uniform sampler2D screenTexture;

void main() {
    vec3 texColor = texture2D(screenTexture, TexCoords).rgb;
    float gray = dot(texColor, vec3(0.299, 0.587, 0.114));
    color = vec4(vec3(gray), 1.0);
}