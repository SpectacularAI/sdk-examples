from OpenGL.GL import *
import numpy as np

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

def lookAt(eye, center, up):
    forward = center - eye
    forward = forward / np.linalg.norm(forward)
    up = up / np.linalg.norm(up)
    right = np.cross(forward, up)
    newUp = np.cross(forward, right)

    # view matrix
    return np.array([
        [right[0], right[1], right[2], -np.dot(right, eye)],
        [newUp[0], newUp[1], newUp[2], -np.dot(newUp, eye)],
        [forward[0], forward[1], forward[2], -np.dot(forward, eye)],
        [0, 0, 0, 1]
    ])

def getOrthographicProjectionMatrixOpenGL(left, right, bottom, top, near, far):
    return np.array([
        [2 / (right - left), 0, 0, -(right + left) / (right - left)],
        [0, -2 / (top - bottom), 0, -(top + bottom) / (top - bottom)],
        [0, 0, 2 / (far - near), -(far + near) / (far - near)],
        [0, 0, 0, 1]
    ], dtype=np.float32)
