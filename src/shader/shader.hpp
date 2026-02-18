#pragma once

#include <string>
#include <unordered_map>
#include <iostream>
#include <glad/gl.h>

class Shader {
public:
    enum class Type {
        VERTEX,
        FRAGMENT,
        PROGRAM
    };

    Shader();
    ~Shader();

    // Disable copy constructor and assignment operator
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    // Enable move constructor and assignment operator
    Shader(Shader&& other) noexcept;
    Shader& operator=(Shader&& other) noexcept;

    // Load and compile shader from source code
    bool loadFromSource(const std::string& vertexSource, const std::string& fragmentSource);
    
    // Load shader from files
    bool loadFromFile(const std::string& vertexPath, const std::string& fragmentPath);

    // Use the shader program
    void use() const;
    
    int getUniformLocation(const std::string& name) const;

    // Get program ID  
    unsigned int getID() const { return programID; }

    // Check if shader is valid
    bool isValid() const { return programID != 0; }

    void render();

    void destroy();

private:
    unsigned int programID;
    mutable std::unordered_map<std::string, int> uniformLocationCache;

    // Helper functions
    unsigned int compileShader(const std::string& source, unsigned int type);
    unsigned int createProgram(unsigned int vertexShader, unsigned int fragmentShader);
    void checkCompileErrors(unsigned int shader, Type type);
    std::string loadFile(const std::string& path);
};