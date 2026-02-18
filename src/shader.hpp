#pragma once

#include <string>
#include <unordered_map>
#include <iostream>
#include "gl_extensions.hpp"

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
    
    // Utility uniform functions
    void setFloat(const std::string& name, float value) const;
    void setInt(const std::string& name, int value) const;
    void setVec2(const std::string& name, float x, float y) const;
    void setVec3(const std::string& name, float x, float y, float z) const;
    void setVec4(const std::string& name, float x, float y, float z, float w) const;
    void setMat4(const std::string& name, const float* value) const;

    // Get program ID  
    unsigned int getID() const { return programID; }

    // Check if shader is valid
    bool isValid() const { return programID != 0; }

private:
    unsigned int programID;
    mutable std::unordered_map<std::string, int> uniformLocationCache;

    // Helper functions
    unsigned int compileShader(const std::string& source, unsigned int type);
    unsigned int createProgram(unsigned int vertexShader, unsigned int fragmentShader);
    int getUniformLocation(const std::string& name) const;
    void checkCompileErrors(unsigned int shader, Type type);
    std::string loadFile(const std::string& path);
};