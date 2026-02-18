#pragma once

#include <GL/gl.h>
#include <GLFW/glfw3.h>

// OpenGL function pointer types
typedef GLuint (*PFNGLCREATESHADERPROC)(GLenum type);
typedef void (*PFNGLSHADERSOURCEPROC)(GLuint shader, GLsizei count, const GLchar *const*string, const GLint *length);
typedef void (*PFNGLCOMPILESHADERPROC)(GLuint shader);
typedef void (*PFNGLGETSHADERIVPROC)(GLuint shader, GLenum pname, GLint *params);
typedef void (*PFNGLGETSHADERINFOLOGPROC)(GLuint shader, GLsizei bufSize, GLsizei *length, GLchar *infoLog);
typedef void (*PFNGLDELETESHADERPROC)(GLuint shader);
typedef GLuint (*PFNGLCREATEPROGRAMPROC)(void);
typedef void (*PFNGLATTACHSHADERPROC)(GLuint program, GLuint shader);
typedef void (*PFNGLLINKPROGRAMPROC)(GLuint program);
typedef void (*PFNGLGETPROGRAMIVPROC)(GLuint program, GLenum pname, GLint *params);
typedef void (*PFNGLGETPROGRAMINFOLOGPROC)(GLuint program, GLsizei bufSize, GLsizei *length, GLchar *infoLog);
typedef void (*PFNGLDELETEPROGRAMPROC)(GLuint program);
typedef void (*PFNGLUSEPROGRAMPROC)(GLuint program);
typedef GLint (*PFNGLGETUNIFORMLOCATIONPROC)(GLuint program, const GLchar *name);
typedef void (*PFNGLUNIFORM1FPROC)(GLint location, GLfloat v0);
typedef void (*PFNGLUNIFORM1IPROC)(GLint location, GLint v0);
typedef void (*PFNGLUNIFORM2FPROC)(GLint location, GLfloat v0, GLfloat v1);
typedef void (*PFNGLUNIFORM3FPROC)(GLint location, GLfloat v0, GLfloat v1, GLfloat v2);
typedef void (*PFNGLUNIFORM4FPROC)(GLint location, GLfloat v0, GLfloat v1, GLfloat v2, GLfloat v3);
typedef void (*PFNGLUNIFORMMATRIX4FVPROC)(GLint location, GLsizei count, GLboolean transpose, const GLfloat *value);

// Framebuffer functions
typedef void (*PFNGLGENFRAMEBUFFERSPROC)(GLsizei n, GLuint *framebuffers);
typedef void (*PFNGLBINDFRAMEBUFFERPROC)(GLenum target, GLuint framebuffer);
typedef void (*PFNGLFRAMEBUFFERTEXTURE2DPROC)(GLenum target, GLenum attachment, GLenum textarget, GLuint texture, GLint level);
typedef GLenum (*PFNGLCHECKFRAMEBUFFERSTATUSPROC)(GLenum target);
typedef void (*PFNGLDELETEFRAMEBUFFERSPROC)(GLsizei n, const GLuint *framebuffers);
typedef void (*PFNGLGENRENDERBUFFERSPROC)(GLsizei n, GLuint *renderbuffers);
typedef void (*PFNGLBINDRENDERBUFFERPROC)(GLenum target, GLuint renderbuffer);
typedef void (*PFNGLRENDERBUFFERSTORAGEPROC)(GLenum target, GLenum internalformat, GLsizei width, GLsizei height);
typedef void (*PFNGLFRAMEBUFFERRENDERBUFFERPROC)(GLenum target, GLenum attachment, GLenum renderbuffertarget, GLuint renderbuffer);
typedef void (*PFNGLDELETERENDERBUFFERSPROC)(GLsizei n, const GLuint *renderbuffers);

// Vertex Array functions
typedef void (*PFNGLGENVERTEXARRAYSPROC)(GLsizei n, GLuint *arrays);
typedef void (*PFNGLBINDVERTEXARRAYPROC)(GLuint array);
typedef void (*PFNGLDELETEVERTEXARRAYSPROC)(GLsizei n, const GLuint *arrays);
typedef void (*PFNGLGENBUFFERSPROC)(GLsizei n, GLuint *buffers);
typedef void (*PFNGLBINDBUFFERPROC)(GLenum target, GLuint buffer);
typedef void (*PFNGLBUFFERDATAPROC)(GLenum target, GLsizeiptr size, const void *data, GLenum usage);
typedef void (*PFNGLDELETEBUFFERSPROC)(GLsizei n, const GLuint *buffers);
typedef void (*PFNGLENABLEVERTEXATTRIBARRAYPROC)(GLuint index);
typedef void (*PFNGLVERTEXATTRIBPOINTERPROC)(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void *pointer);
typedef void (*PFNGLACTIVETEXTUREPROC)(GLenum texture);

// OpenGL constants
#ifndef GL_VERTEX_SHADER
#define GL_VERTEX_SHADER 0x8B31
#endif
#ifndef GL_FRAGMENT_SHADER
#define GL_FRAGMENT_SHADER 0x8B30
#endif
#ifndef GL_COMPILE_STATUS
#define GL_COMPILE_STATUS 0x8B81
#endif
#ifndef GL_LINK_STATUS
#define GL_LINK_STATUS 0x8B82
#endif
#ifndef GL_FRAMEBUFFER
#define GL_FRAMEBUFFER 0x8D40
#endif
#ifndef GL_COLOR_ATTACHMENT0
#define GL_COLOR_ATTACHMENT0 0x8CE0
#endif
#ifndef GL_DEPTH_ATTACHMENT
#define GL_DEPTH_ATTACHMENT 0x8D00
#endif
#ifndef GL_RENDERBUFFER
#define GL_RENDERBUFFER 0x8D41
#endif
#ifndef GL_DEPTH_COMPONENT
#define GL_DEPTH_COMPONENT 0x1902
#endif
#ifndef GL_FRAMEBUFFER_COMPLETE
#define GL_FRAMEBUFFER_COMPLETE 0x8CD5
#endif
#ifndef GL_ARRAY_BUFFER
#define GL_ARRAY_BUFFER 0x8892
#endif
#ifndef GL_STATIC_DRAW
#define GL_STATIC_DRAW 0x88E4
#endif
#ifndef GL_TEXTURE0
#define GL_TEXTURE0 0x84C0
#endif

class GLExtensionLoader {
public:
    static bool loadExtensions();
    
    // Shader functions
    static PFNGLCREATESHADERPROC glCreateShader;
    static PFNGLSHADERSOURCEPROC glShaderSource;
    static PFNGLCOMPILESHADERPROC glCompileShader;
    static PFNGLGETSHADERIVPROC glGetShaderiv;
    static PFNGLGETSHADERINFOLOGPROC glGetShaderInfoLog;
    static PFNGLDELETESHADERPROC glDeleteShader;
    static PFNGLCREATEPROGRAMPROC glCreateProgram;
    static PFNGLATTACHSHADERPROC glAttachShader;
    static PFNGLLINKPROGRAMPROC glLinkProgram;
    static PFNGLGETPROGRAMIVPROC glGetProgramiv;
    static PFNGLGETPROGRAMINFOLOGPROC glGetProgramInfoLog;
    static PFNGLDELETEPROGRAMPROC glDeleteProgram;
    static PFNGLUSEPROGRAMPROC glUseProgram;
    static PFNGLGETUNIFORMLOCATIONPROC glGetUniformLocation;
    static PFNGLUNIFORM1FPROC glUniform1f;
    static PFNGLUNIFORM1IPROC glUniform1i;
    static PFNGLUNIFORM2FPROC glUniform2f;
    static PFNGLUNIFORM3FPROC glUniform3f;
    static PFNGLUNIFORM4FPROC glUniform4f;
    static PFNGLUNIFORMMATRIX4FVPROC glUniformMatrix4fv;
    
    // Framebuffer functions
    static PFNGLGENFRAMEBUFFERSPROC glGenFramebuffers;
    static PFNGLBINDFRAMEBUFFERPROC glBindFramebuffer;
    static PFNGLFRAMEBUFFERTEXTURE2DPROC glFramebufferTexture2D;
    static PFNGLCHECKFRAMEBUFFERSTATUSPROC glCheckFramebufferStatus;
    static PFNGLDELETEFRAMEBUFFERSPROC glDeleteFramebuffers;
    static PFNGLGENRENDERBUFFERSPROC glGenRenderbuffers;
    static PFNGLBINDRENDERBUFFERPROC glBindRenderbuffer;
    static PFNGLRENDERBUFFERSTORAGEPROC glRenderbufferStorage;
    static PFNGLFRAMEBUFFERRENDERBUFFERPROC glFramebufferRenderbuffer;
    static PFNGLDELETERENDERBUFFERSPROC glDeleteRenderbuffers;
    
    // Vertex Array functions
    static PFNGLGENVERTEXARRAYSPROC glGenVertexArrays;
    static PFNGLBINDVERTEXARRAYPROC glBindVertexArray;
    static PFNGLDELETEVERTEXARRAYSPROC glDeleteVertexArrays;
    static PFNGLGENBUFFERSPROC glGenBuffers;
    static PFNGLBINDBUFFERPROC glBindBuffer;
    static PFNGLBUFFERDATAPROC glBufferData;
    static PFNGLDELETEBUFFERSPROC glDeleteBuffers;
    static PFNGLENABLEVERTEXATTRIBARRAYPROC glEnableVertexAttribArray;
    static PFNGLVERTEXATTRIBPOINTERPROC glVertexAttribPointer;
    static PFNGLACTIVETEXTUREPROC glActiveTexture;
    
private:
    static bool extensionsLoaded;
};