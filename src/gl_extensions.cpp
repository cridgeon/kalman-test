#include "gl_extensions.hpp"
#include <iostream>

bool GLExtensionLoader::extensionsLoaded = false;

// Initialize all function pointers to nullptr
PFNGLCREATESHADERPROC GLExtensionLoader::glCreateShader = nullptr;
PFNGLSHADERSOURCEPROC GLExtensionLoader::glShaderSource = nullptr;
PFNGLCOMPILESHADERPROC GLExtensionLoader::glCompileShader = nullptr;
PFNGLGETSHADERIVPROC GLExtensionLoader::glGetShaderiv = nullptr;
PFNGLGETSHADERINFOLOGPROC GLExtensionLoader::glGetShaderInfoLog = nullptr;
PFNGLDELETESHADERPROC GLExtensionLoader::glDeleteShader = nullptr;
PFNGLCREATEPROGRAMPROC GLExtensionLoader::glCreateProgram = nullptr;
PFNGLATTACHSHADERPROC GLExtensionLoader::glAttachShader = nullptr;
PFNGLLINKPROGRAMPROC GLExtensionLoader::glLinkProgram = nullptr;
PFNGLGETPROGRAMIVPROC GLExtensionLoader::glGetProgramiv = nullptr;
PFNGLGETPROGRAMINFOLOGPROC GLExtensionLoader::glGetProgramInfoLog = nullptr;
PFNGLDELETEPROGRAMPROC GLExtensionLoader::glDeleteProgram = nullptr;
PFNGLUSEPROGRAMPROC GLExtensionLoader::glUseProgram = nullptr;
PFNGLGETUNIFORMLOCATIONPROC GLExtensionLoader::glGetUniformLocation = nullptr;
PFNGLUNIFORM1FPROC GLExtensionLoader::glUniform1f = nullptr;
PFNGLUNIFORM1IPROC GLExtensionLoader::glUniform1i = nullptr;
PFNGLUNIFORM2FPROC GLExtensionLoader::glUniform2f = nullptr;
PFNGLUNIFORM3FPROC GLExtensionLoader::glUniform3f = nullptr;
PFNGLUNIFORM4FPROC GLExtensionLoader::glUniform4f = nullptr;
PFNGLUNIFORMMATRIX4FVPROC GLExtensionLoader::glUniformMatrix4fv = nullptr;

PFNGLGENFRAMEBUFFERSPROC GLExtensionLoader::glGenFramebuffers = nullptr;
PFNGLBINDFRAMEBUFFERPROC GLExtensionLoader::glBindFramebuffer = nullptr;
PFNGLFRAMEBUFFERTEXTURE2DPROC GLExtensionLoader::glFramebufferTexture2D = nullptr;
PFNGLCHECKFRAMEBUFFERSTATUSPROC GLExtensionLoader::glCheckFramebufferStatus = nullptr;
PFNGLDELETEFRAMEBUFFERSPROC GLExtensionLoader::glDeleteFramebuffers = nullptr;
PFNGLGENRENDERBUFFERSPROC GLExtensionLoader::glGenRenderbuffers = nullptr;
PFNGLBINDRENDERBUFFERPROC GLExtensionLoader::glBindRenderbuffer = nullptr;
PFNGLRENDERBUFFERSTORAGEPROC GLExtensionLoader::glRenderbufferStorage = nullptr;
PFNGLFRAMEBUFFERRENDERBUFFERPROC GLExtensionLoader::glFramebufferRenderbuffer = nullptr;
PFNGLDELETERENDERBUFFERSPROC GLExtensionLoader::glDeleteRenderbuffers = nullptr;

PFNGLGENVERTEXARRAYSPROC GLExtensionLoader::glGenVertexArrays = nullptr;
PFNGLBINDVERTEXARRAYPROC GLExtensionLoader::glBindVertexArray = nullptr;
PFNGLDELETEVERTEXARRAYSPROC GLExtensionLoader::glDeleteVertexArrays = nullptr;
PFNGLGENBUFFERSPROC GLExtensionLoader::glGenBuffers = nullptr;
PFNGLBINDBUFFERPROC GLExtensionLoader::glBindBuffer = nullptr;
PFNGLBUFFERDATAPROC GLExtensionLoader::glBufferData = nullptr;
PFNGLDELETEBUFFERSPROC GLExtensionLoader::glDeleteBuffers = nullptr;
PFNGLENABLEVERTEXATTRIBARRAYPROC GLExtensionLoader::glEnableVertexAttribArray = nullptr;
PFNGLVERTEXATTRIBPOINTERPROC GLExtensionLoader::glVertexAttribPointer = nullptr;
PFNGLACTIVETEXTUREPROC GLExtensionLoader::glActiveTexture = nullptr;

bool GLExtensionLoader::loadExtensions() {
    if (extensionsLoaded) return true;

    // Load shader functions
    glCreateShader = reinterpret_cast<PFNGLCREATESHADERPROC>(glfwGetProcAddress("glCreateShader"));
    glShaderSource = reinterpret_cast<PFNGLSHADERSOURCEPROC>(glfwGetProcAddress("glShaderSource"));
    glCompileShader = reinterpret_cast<PFNGLCOMPILESHADERPROC>(glfwGetProcAddress("glCompileShader"));
    glGetShaderiv = reinterpret_cast<PFNGLGETSHADERIVPROC>(glfwGetProcAddress("glGetShaderiv"));
    glGetShaderInfoLog = reinterpret_cast<PFNGLGETSHADERINFOLOGPROC>(glfwGetProcAddress("glGetShaderInfoLog"));
    glDeleteShader = reinterpret_cast<PFNGLDELETESHADERPROC>(glfwGetProcAddress("glDeleteShader"));
    glCreateProgram = reinterpret_cast<PFNGLCREATEPROGRAMPROC>(glfwGetProcAddress("glCreateProgram"));
    glAttachShader = reinterpret_cast<PFNGLATTACHSHADERPROC>(glfwGetProcAddress("glAttachShader"));
    glLinkProgram = reinterpret_cast<PFNGLLINKPROGRAMPROC>(glfwGetProcAddress("glLinkProgram"));
    glGetProgramiv = reinterpret_cast<PFNGLGETPROGRAMIVPROC>(glfwGetProcAddress("glGetProgramiv"));
    glGetProgramInfoLog = reinterpret_cast<PFNGLGETPROGRAMINFOLOGPROC>(glfwGetProcAddress("glGetProgramInfoLog"));
    glDeleteProgram = reinterpret_cast<PFNGLDELETEPROGRAMPROC>(glfwGetProcAddress("glDeleteProgram"));
    glUseProgram = reinterpret_cast<PFNGLUSEPROGRAMPROC>(glfwGetProcAddress("glUseProgram"));
    glGetUniformLocation = reinterpret_cast<PFNGLGETUNIFORMLOCATIONPROC>(glfwGetProcAddress("glGetUniformLocation"));
    glUniform1f = reinterpret_cast<PFNGLUNIFORM1FPROC>(glfwGetProcAddress("glUniform1f"));
    glUniform1i = reinterpret_cast<PFNGLUNIFORM1IPROC>(glfwGetProcAddress("glUniform1i"));
    glUniform2f = reinterpret_cast<PFNGLUNIFORM2FPROC>(glfwGetProcAddress("glUniform2f"));
    glUniform3f = reinterpret_cast<PFNGLUNIFORM3FPROC>(glfwGetProcAddress("glUniform3f"));
    glUniform4f = reinterpret_cast<PFNGLUNIFORM4FPROC>(glfwGetProcAddress("glUniform4f"));
    glUniformMatrix4fv = reinterpret_cast<PFNGLUNIFORMMATRIX4FVPROC>(glfwGetProcAddress("glUniformMatrix4fv"));

    // Load framebuffer functions
    glGenFramebuffers = reinterpret_cast<PFNGLGENFRAMEBUFFERSPROC>(glfwGetProcAddress("glGenFramebuffers"));
    glBindFramebuffer = reinterpret_cast<PFNGLBINDFRAMEBUFFERPROC>(glfwGetProcAddress("glBindFramebuffer"));
    glFramebufferTexture2D = reinterpret_cast<PFNGLFRAMEBUFFERTEXTURE2DPROC>(glfwGetProcAddress("glFramebufferTexture2D"));
    glCheckFramebufferStatus = reinterpret_cast<PFNGLCHECKFRAMEBUFFERSTATUSPROC>(glfwGetProcAddress("glCheckFramebufferStatus"));
    glDeleteFramebuffers = reinterpret_cast<PFNGLDELETEFRAMEBUFFERSPROC>(glfwGetProcAddress("glDeleteFramebuffers"));
    glGenRenderbuffers = reinterpret_cast<PFNGLGENRENDERBUFFERSPROC>(glfwGetProcAddress("glGenRenderbuffers"));
    glBindRenderbuffer = reinterpret_cast<PFNGLBINDRENDERBUFFERPROC>(glfwGetProcAddress("glBindRenderbuffer"));
    glRenderbufferStorage = reinterpret_cast<PFNGLRENDERBUFFERSTORAGEPROC>(glfwGetProcAddress("glRenderbufferStorage"));
    glFramebufferRenderbuffer = reinterpret_cast<PFNGLFRAMEBUFFERRENDERBUFFERPROC>(glfwGetProcAddress("glFramebufferRenderbuffer"));
    glDeleteRenderbuffers = reinterpret_cast<PFNGLDELETERENDERBUFFERSPROC>(glfwGetProcAddress("glDeleteRenderbuffers"));

    // Load vertex array functions
    glGenVertexArrays = reinterpret_cast<PFNGLGENVERTEXARRAYSPROC>(glfwGetProcAddress("glGenVertexArrays"));
    glBindVertexArray = reinterpret_cast<PFNGLBINDVERTEXARRAYPROC>(glfwGetProcAddress("glBindVertexArray"));
    glDeleteVertexArrays = reinterpret_cast<PFNGLDELETEVERTEXARRAYSPROC>(glfwGetProcAddress("glDeleteVertexArrays"));
    glGenBuffers = reinterpret_cast<PFNGLGENBUFFERSPROC>(glfwGetProcAddress("glGenBuffers"));
    glBindBuffer = reinterpret_cast<PFNGLBINDBUFFERPROC>(glfwGetProcAddress("glBindBuffer"));
    glBufferData = reinterpret_cast<PFNGLBUFFERDATAPROC>(glfwGetProcAddress("glBufferData"));
    glDeleteBuffers = reinterpret_cast<PFNGLDELETEBUFFERSPROC>(glfwGetProcAddress("glDeleteBuffers"));
    glEnableVertexAttribArray = reinterpret_cast<PFNGLENABLEVERTEXATTRIBARRAYPROC>(glfwGetProcAddress("glEnableVertexAttribArray"));
    glVertexAttribPointer = reinterpret_cast<PFNGLVERTEXATTRIBPOINTERPROC>(glfwGetProcAddress("glVertexAttribPointer"));
    glActiveTexture = reinterpret_cast<PFNGLACTIVETEXTUREPROC>(glfwGetProcAddress("glActiveTexture"));

    // Check if critical functions loaded successfully
    if (!glCreateShader || !glShaderSource || !glCompileShader || !glCreateProgram || 
        !glLinkProgram || !glUseProgram || !glGetUniformLocation ||
        !glGenFramebuffers || !glBindFramebuffer || !glGenVertexArrays || !glGenBuffers) {
        std::cerr << "Failed to load required OpenGL extensions" << std::endl;
        return false;
    }

    extensionsLoaded = true;
    return true;
}