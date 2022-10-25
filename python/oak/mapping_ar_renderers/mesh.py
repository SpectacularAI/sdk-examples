import pathlib

import numpy as np

from OpenGL.GL import * # all prefixed with gl so OK to import *

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

class MeshProgram:
    def __init__(self, program):
        assert(program)
        self.program = program
        self.attributeVertex = glGetAttribLocation(self.program, "a_Position")
        self.attributeTexCoord = glGetAttribLocation(self.program, "a_TexCoord")
        self.attributeNormal = glGetAttribLocation(self.program, "a_Normal")
        self.uniformModelViewProjection = glGetUniformLocation(self.program, "u_ModelViewProjection")

class MeshRenderer:
    modelViewProjection = None
    meshProgram = None

    # 1d arrays of GLfloat.
    vertexData = np.array([])
    normalData = np.array([])
    texCoordData = np.array([])

    def __init__(self):
        assetDir = pathlib.Path(__file__).resolve().parent
        meshVert = (assetDir / "mesh.vert").read_text()
        meshFrag = (assetDir / "mesh.frag").read_text()
        self.meshProgram = MeshProgram(createProgram(meshVert, meshFrag))

    def render(self):
        if self.modelViewProjection is None: return
        if self.vertexData.shape[0] == 0: return

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        glUseProgram(self.meshProgram.program)
        glUniformMatrix4fv(self.meshProgram.uniformModelViewProjection, 1, GL_FALSE, self.modelViewProjection.transpose())

        coordsPerVertex = 3
        glEnableVertexAttribArray(self.meshProgram.attributeVertex)
        glVertexAttribPointer(self.meshProgram.attributeVertex, coordsPerVertex,
            GL_FLOAT, GL_FALSE, 0, self.vertexData)

        glEnableVertexAttribArray(self.meshProgram.attributeTexCoord)
        coordsPerTex = 3
        glVertexAttribPointer(self.meshProgram.attributeTexCoord, coordsPerTex, GL_FLOAT, GL_FALSE, 0, self.texCoordData)

        glEnableVertexAttribArray(self.meshProgram.attributeNormal)
        glVertexAttribPointer(self.meshProgram.attributeNormal, coordsPerVertex, GL_FLOAT, GL_FALSE, 0,
                              self.normalData)

        nVertices = self.vertexData.shape[0] // coordsPerVertex
        glDrawArrays(GL_TRIANGLES, 0, nVertices)

        glDisableVertexAttribArray(self.meshProgram.attributeNormal)
        glDisableVertexAttribArray(self.meshProgram.attributeTexCoord)
        glDisableVertexAttribArray(self.meshProgram.attributeVertex)
        glUseProgram(0)

        glDisable(GL_BLEND)

    def setMesh(self, mesh):
        if mesh is None: return
        if not mesh.hasNormals(): return

        vertexPositions = mesh.getPositionData()
        vertexInds = mesh.getFaceVertices().flatten()
        self.vertexData = vertexPositions[vertexInds, :].flatten()

        normals = mesh.getNormalData()
        normalInds = mesh.getFaceNormals().flatten()
        self.normalData = normals[normalInds, :].flatten()

        self.texCoordData = np.array([])
        n = len(vertexInds) // 3
        c = 1.0 # Could be used for uncertainty.
        self.texCoordData = np.zeros(3 * len(vertexInds))
        for i in range(n):
            self.texCoordData[9 * i + 3] = 1
            self.texCoordData[9 * i + 7] = 1
            self.texCoordData[9 * i + 2] = c
            self.texCoordData[9 * i + 5] = c
            self.texCoordData[9 * i + 8] = c

    def setPose(self, cameraPose):
        near, far = 0.01, 100.0
        projection = cameraPose.camera.getProjectionMatrixOpenGL(near, far)
        modelView = cameraPose.getWorldToCameraMatrix()
        self.modelViewProjection = projection @ modelView
