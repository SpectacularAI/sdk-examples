import numpy as np
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

# Class that mocks `spectacularAI::mapping::Mesh`.
class Mesh:
    def __init__(self):
        self.vertexPositions = np.empty((0, 3), float)
        self.normals = np.empty((0, 3), float)
        self.faceVertices = np.empty((0, 3), int)
        self.faceNormals = np.empty((0, 3), int)

    def hasNormals(self):
        return self.normals.shape[0] > 0

    def getPositionData(self):
        return self.vertexPositions

    def getNormalData(self):
        return self.normals

    def getFaceVertices(self):
        return self.faceVertices

    def getFaceNormals(self):
        return self.faceNormals

def loadObjToMesh(objPath, objPosition):
    mesh = Mesh()
    with open(objPath, 'r') as objFile:
        for line in objFile:
            if line.startswith('#'): continue
            cmd, _, rest = line.partition(' ')
            data = rest.split()
            if cmd == 'v':
                mesh.vertexPositions = np.vstack((mesh.vertexPositions, [float(c) for c in data]))
            elif cmd == 'vn':
                mesh.normals = np.vstack((mesh.normals, [float(c) for c in data]))
            elif cmd == 'f':
                v = []
                n = []
                for i, token in enumerate(data):
                    indices = token.split('/')
                    assert(int(indices[0]) >= 1) # Negative indices counting from the end are not handled.
                    v.append(int(indices[0]) - 1)
                    # Textures not handled.
                    if len(indices) >= 3: n.append(int(indices[2]) - 1)

                    if i < 2: continue
                    # If i > 2, interpret the polygon as triangle fan and hope for the best.
                    mesh.faceVertices = np.vstack((mesh.faceVertices, [v[0], v[-2], v[-1]]))
                    if n: mesh.faceNormals = np.vstack((mesh.faceNormals, [n[0], n[-2], n[-1]]))

    mesh.vertexPositions += objPosition

    return mesh
