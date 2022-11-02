from OpenGL.GL import *

def loadShader(shaderType, shaderSource):
    shader = glCreateShader(shaderType)
    assert(shader)
    glShaderSource(shader, shaderSource)
    glCompileShader(shader)
    result = glGetShaderiv(shader, GL_COMPILE_STATUS)
    if not result: raise RuntimeError(glGetShaderInfoLog(shader))
    return shader

def createProgram(vertexSource, fragmentSource):
    vertexShader = loadShader(GL_VERTEX_SHADER, vertexSource)
    fragmentShader = loadShader(GL_FRAGMENT_SHADER, fragmentSource)
    program = glCreateProgram()
    assert(program)
    glAttachShader(program, vertexShader)
    glAttachShader(program, fragmentShader)
    glLinkProgram(program)
    result = glGetProgramiv(program, GL_LINK_STATUS)
    if not result: raise RuntimeError(glGetProgramInfoLog(program))
    return program
