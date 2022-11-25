import pathlib

import numpy as np

from OpenGL.GL import *

from .util import *

class MeshProgram:
    def __init__(self, program):
        assert(program)
        self.program = program
        self.attributeVertex = glGetAttribLocation(self.program, "a_Position")
        self.attributeTexCoord = glGetAttribLocation(self.program, "a_TexCoord")
        self.attributeNormal = glGetAttribLocation(self.program, "a_Normal")
        self.uniformModelViewProjection = glGetUniformLocation(self.program, "u_ModelViewProjection")
        self.uniformOptions = glGetUniformLocation(self.program, "u_Options")

class MeshRenderer:
    # Value greater than zero enables at index:
    # 0) Show borders.
    # 1) Enable "fake mesh". The number sets density of the lines.
    # 2) Fake mesh z-levels only.
    # 3) Alpha multiplier.
    # 4) Show camera view.
    # 5) Show mesh.
    OPTIONS = [
        [0., 0., 0., 0.75, 1., 1.],
        [1., 0., 0., 0.75, 1., 1.],
        [1., 10., 0., 1., 1., 1.],
        [1., 10., 1., 1., 1., 1.],
        [1., 0., 0., 0.75, 0., 1.],
    ]
    selectedOption = 0

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

    def __init_tex_coord_data(self, nFaces):
        c = 1.0 # Could be used for controlling alpha.
        faceTexCoords = np.array([0, 0, c, 1, 0, c, 0, 1, c])
        self.texCoordData = np.tile(faceTexCoords, nFaces)

    def render(self):
        if self.modelViewProjection is None: return
        if self.vertexData.shape[0] == 0: return

        op = MeshRenderer.OPTIONS[self.selectedOption]
        if op[5] == 0.0: return

        glEnable(GL_DEPTH_TEST)
        glDepthMask(GL_TRUE)
        glClear(GL_DEPTH_BUFFER_BIT)

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        if op[4] == 0.0: glClear(GL_COLOR_BUFFER_BIT)

        glUseProgram(self.meshProgram.program)
        glUniform4fv(self.meshProgram.uniformOptions, 1, np.array(op[:4]))
        glUniformMatrix4fv(self.meshProgram.uniformModelViewProjection, 1, GL_FALSE, self.modelViewProjection.transpose())

        coordsPerVertex = 3
        glEnableVertexAttribArray(self.meshProgram.attributeVertex)
        glVertexAttribPointer(self.meshProgram.attributeVertex, coordsPerVertex,
            GL_FLOAT, GL_FALSE, 0, self.vertexData)

        glEnableVertexAttribArray(self.meshProgram.attributeTexCoord)
        coordsPerTex = 3
        n = len(self.vertexData)
        glVertexAttribPointer(self.meshProgram.attributeTexCoord, coordsPerTex, GL_FLOAT, GL_FALSE, 0, self.texCoordData[:n])

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
        glDisable(GL_DEPTH_TEST)

    def setMesh(self, mesh):
        if mesh is None: return
        if not mesh.hasNormals(): return

        vertexPositions = mesh.getPositionData()
        vertexInds = mesh.getFaceVertices().flatten()
        self.vertexData = vertexPositions[vertexInds, :].flatten()

        normals = mesh.getNormalData()
        normalInds = mesh.getFaceNormals().flatten()
        self.normalData = normals[normalInds, :].flatten()

        if len(self.texCoordData) < len(self.vertexData):
            # Optimization: (in this demo texture coordinates are a constant repeating array)
            # Double the tex coordinate array size when the old one is too small.
            self.__init_tex_coord_data(2 * len(self.vertexData))

    def setPose(self, cameraPose):
        near, far = 0.01, 100.0
        projection = cameraPose.camera.getProjectionMatrixOpenGL(near, far)
        modelView = cameraPose.getWorldToCameraMatrix()
        self.modelViewProjection = projection @ modelView

    def nextMode(self):
        self.selectedOption = (self.selectedOption + 1) % len(MeshRenderer.OPTIONS)
