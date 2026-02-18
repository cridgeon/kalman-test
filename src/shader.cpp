#include "shader.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

Shader::Shader() : programID(0) {}

Shader::~Shader() {
    if (programID != 0) {
        GLExtensionLoader::glDeleteProgram(programID);
    }
}

Shader::Shader(Shader&& other) noexcept : programID(other.programID), uniformLocationCache(std::move(other.uniformLocationCache)) {
    other.programID = 0;
}

Shader& Shader::operator=(Shader&& other) noexcept {
    if (this != &other) {
        if (programID != 0) {
            GLExtensionLoader::glDeleteProgram(programID);
        }
        programID = other.programID;
        uniformLocationCache = std::move(other.uniformLocationCache);
        other.programID = 0;
    }
    return *this;
}

bool Shader::loadFromSource(const std::string& vertexSource, const std::string& fragmentSource) {
    unsigned int vertexShader = compileShader(vertexSource, GL_VERTEX_SHADER);
    if (vertexShader == 0) return false;

    unsigned int fragmentShader = compileShader(fragmentSource, GL_FRAGMENT_SHADER);
    if (fragmentShader == 0) {
        GLExtensionLoader::glDeleteShader(vertexShader);
        return false;
    }

    programID = createProgram(vertexShader, fragmentShader);
    
    GLExtensionLoader::glDeleteShader(vertexShader);
    GLExtensionLoader::glDeleteShader(fragmentShader);
    
    return programID != 0;
}

bool Shader::loadFromFile(const std::string& vertexPath, const std::string& fragmentPath) {
    std::string vertexSource = loadFile(vertexPath);
    std::string fragmentSource = loadFile(fragmentPath);
    
    if (vertexSource.empty() || fragmentSource.empty()) {
        return false;
    }
    
    return loadFromSource(vertexSource, fragmentSource);
}

void Shader::use() const {
    if (programID != 0) {
        GLExtensionLoader::glUseProgram(programID);
    }
}

void Shader::setFloat(const std::string& name, float value) const {
    GLExtensionLoader::glUniform1f(getUniformLocation(name), value);
}

void Shader::setInt(const std::string& name, int value) const {
    GLExtensionLoader::glUniform1i(getUniformLocation(name), value);
}

void Shader::setVec2(const std::string& name, float x, float y) const {
    GLExtensionLoader::glUniform2f(getUniformLocation(name), x, y);
}

void Shader::setVec3(const std::string& name, float x, float y, float z) const {
    GLExtensionLoader::glUniform3f(getUniformLocation(name), x, y, z);
}

void Shader::setVec4(const std::string& name, float x, float y, float z, float w) const {
    GLExtensionLoader::glUniform4f(getUniformLocation(name), x, y, z, w);
}

void Shader::setMat4(const std::string& name, const float* value) const {
    GLExtensionLoader::glUniformMatrix4fv(getUniformLocation(name), 1, GL_FALSE, value);
}

unsigned int Shader::compileShader(const std::string& source, unsigned int type) {
    unsigned int shader = GLExtensionLoader::glCreateShader(type);
    const char* src = source.c_str();
    GLExtensionLoader::glShaderSource(shader, 1, &src, nullptr);
    GLExtensionLoader::glCompileShader(shader);
    
    checkCompileErrors(shader, type == GL_VERTEX_SHADER ? Type::VERTEX : Type::FRAGMENT);
    
    int success;
    GLExtensionLoader::glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        GLExtensionLoader::glDeleteShader(shader);
        return 0;
    }
    
    return shader;
}

unsigned int Shader::createProgram(unsigned int vertexShader, unsigned int fragmentShader) {
    unsigned int program = GLExtensionLoader::glCreateProgram();
    GLExtensionLoader::glAttachShader(program, vertexShader);
    GLExtensionLoader::glAttachShader(program, fragmentShader);
    GLExtensionLoader::glLinkProgram(program);
    
    checkCompileErrors(program, Type::PROGRAM);
    
    int success;
    GLExtensionLoader::glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        GLExtensionLoader::glDeleteProgram(program);
        return 0;
    }
    
    return program;
}

int Shader::getUniformLocation(const std::string& name) const {
    auto it = uniformLocationCache.find(name);
    if (it != uniformLocationCache.end()) {
        return it->second;
    }
    
    int location = GLExtensionLoader::glGetUniformLocation(programID, name.c_str());
    uniformLocationCache[name] = location;
    return location;
}

void Shader::checkCompileErrors(unsigned int shader, Type type) {
    int success;
    char infoLog[1024];
    
    if (type != Type::PROGRAM) {
        GLExtensionLoader::glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            GLExtensionLoader::glGetShaderInfoLog(shader, 1024, nullptr, infoLog);
            std::cerr << "ERROR::SHADER_COMPILATION_ERROR of type: " 
                      << (type == Type::VERTEX ? "VERTEX" : "FRAGMENT") << "\n" 
                      << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
        }
    } else {
        GLExtensionLoader::glGetProgramiv(shader, GL_LINK_STATUS, &success);
        if (!success) {
            GLExtensionLoader::glGetProgramInfoLog(shader, 1024, nullptr, infoLog);
            std::cerr << "ERROR::PROGRAM_LINKING_ERROR\n" 
                      << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
        }
    }
}

std::string Shader::loadFile(const std::string& path) {
    std::string content;
    std::ifstream file;
    
    file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try {
        file.open(path);
        std::stringstream stream;
        stream << file.rdbuf();
        file.close();
        content = stream.str();
    } catch (std::ifstream::failure& e) {
        std::cerr << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ: " << path << std::endl;
        return "";
    }
    
    return content;
}