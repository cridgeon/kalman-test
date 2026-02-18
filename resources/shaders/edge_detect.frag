#version 130

in vec2 TexCoords;
out vec4 color;

uniform sampler2D screenTexture;
uniform vec2 resolution;

void main() {
    vec2 texelSize = 1.0 / resolution;
    
    // Sobel edge detection kernel
    vec3 top         = texture2D(screenTexture, TexCoords + vec2(0.0, texelSize.y)).rgb;
    vec3 bottom      = texture2D(screenTexture, TexCoords + vec2(0.0, -texelSize.y)).rgb;
    vec3 left        = texture2D(screenTexture, TexCoords + vec2(-texelSize.x, 0.0)).rgb;
    vec3 right       = texture2D(screenTexture, TexCoords + vec2(texelSize.x, 0.0)).rgb;
    vec3 topLeft     = texture2D(screenTexture, TexCoords + vec2(-texelSize.x, texelSize.y)).rgb;
    vec3 topRight    = texture2D(screenTexture, TexCoords + vec2(texelSize.x, texelSize.y)).rgb;
    vec3 bottomLeft  = texture2D(screenTexture, TexCoords + vec2(-texelSize.x, -texelSize.y)).rgb;
    vec3 bottomRight = texture2D(screenTexture, TexCoords + vec2(texelSize.x, -texelSize.y)).rgb;
    
    vec3 sx = (topRight + 2.0 * right + bottomRight) - (topLeft + 2.0 * left + bottomLeft);
    vec3 sy = (topLeft + 2.0 * top + topRight) - (bottomLeft + 2.0 * bottom + bottomRight);
    vec3 sobel = sqrt(sx * sx + sy * sy);
    
    color = vec4(sobel, 1.0);
}