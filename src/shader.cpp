#include "shader.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

Shader::Shader() : programID(0) {}

Shader::~Shader() {
    if (programID != 0) {
        glDeleteProgram(programID);
    }
}

Shader::Shader(Shader&& other) noexcept : programID(other.programID), uniformLocationCache(std::move(other.uniformLocationCache)) {
    other.programID = 0;
}

Shader& Shader::operator=(Shader&& other) noexcept {
    if (this != &other) {
        if (programID != 0) {
            glDeleteProgram(programID);
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
        glDeleteShader(vertexShader);
        return false;
    }

    programID = createProgram(vertexShader, fragmentShader);
    
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    
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
        glUseProgram(programID);
    }
}

void Shader::render() {
    // Render fullscreen quad
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
    
    glDisable(GL_BLEND);
}

unsigned int Shader::compileShader(const std::string &source, unsigned int type)
{
    unsigned int shader = glCreateShader(type);
    const char* src = source.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);
    
    checkCompileErrors(shader, type == GL_VERTEX_SHADER ? Type::VERTEX : Type::FRAGMENT);
    
    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glDeleteShader(shader);
        return 0;
    }
    
    return shader;
}

unsigned int Shader::createProgram(unsigned int vertexShader, unsigned int fragmentShader) {
    unsigned int program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);
    
    checkCompileErrors(program, Type::PROGRAM);
    
    int success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        glDeleteProgram(program);
        return 0;
    }
    
    return program;
}

int Shader::getUniformLocation(const std::string& name) const {
    auto it = uniformLocationCache.find(name);
    if (it != uniformLocationCache.end()) {
        return it->second;
    }
    
    int location = glGetUniformLocation(programID, name.c_str());
    uniformLocationCache[name] = location;
    return location;
}

void Shader::checkCompileErrors(unsigned int shader, Type type) {
    int success;
    char infoLog[1024];
    
    if (type != Type::PROGRAM) {
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(shader, 1024, nullptr, infoLog);
            std::cerr << "ERROR::SHADER_COMPILATION_ERROR of type: " 
                      << (type == Type::VERTEX ? "VERTEX" : "FRAGMENT") << "\n" 
                      << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
        }
    } else {
        glGetProgramiv(shader, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(shader, 1024, nullptr, infoLog);
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