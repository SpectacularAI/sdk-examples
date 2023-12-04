import pathlib
import numpy as np
from OpenGL.GL import *
from .util import *

COORDS_PER_VERTEX = 3
COLORS_PER_VERTEX = 3

class PointCloud:
    def __init__(self, vertices, colors=None, normals=None):
        # 1d arrays of GLfloat.
        self.vertices = vertices.flatten() if vertices is not None else None
        self.colors = colors.flatten() if colors is not None else None # optional
        self.normals = normals.flatten() if normals is not None else None # optional

    def size(self):
        return self.vertices.shape[0] // COORDS_PER_VERTEX

    # TODO: slow, fix
    def downsample(self, voxelSize):
        voxelGrid = {}
        for i in range(0, self.size()):
            idx = i * COORDS_PER_VERTEX
            point = self.vertices[idx:idx+COORDS_PER_VERTEX]
            cellIndex = tuple(((point) / voxelSize).astype(int))
            if cellIndex not in voxelGrid:
                voxelGrid[cellIndex] = [idx, idx+1, idx+2]
        indices = np.array(list(voxelGrid.values())).flatten()
        self.vertices = self.vertices[indices]
        if self.colors is not None: self.colors = self.colors[indices]
        if self.normals is not None: self.normals = self.normals[indices]

class PointCloudProgram:
    def __init__(self, program):
        assert(program)
        self.program = program
        self.uniformModel = glGetUniformLocation(self.program, "u_Model")
        self.uniformView = glGetUniformLocation(self.program, "u_View")
        self.uniformProjection = glGetUniformLocation(self.program, "u_Projection")
        self.uniformCameraPositionWorld = glGetUniformLocation(self.program, "u_CameraPositionWorld")
        self.uniformPointSize = glGetUniformLocation(self.program, "u_PointSize")
        self.uniformOpacity = glGetUniformLocation(self.program, "u_Opacity")
        self.uniformColorMode = glGetUniformLocation(self.program, "u_ColorMode")
        self.uniformHasColor = glGetUniformLocation(self.program, "u_HasColor")
        self.uniformHasNormal = glGetUniformLocation(self.program, "u_HasNormal")

class PointCloudRenderer:
    def __init__(self, pointCloud=None, pointSize=1.0, opacity=1.0, maxZ=None, colorMapScale=1.0):
        self.pcProgram = None
        self.pointCloud = pointCloud
        self.vboPosition = None
        self.vboColor = None
        self.vboNormal = None
        self.updated = pointCloud is not None
        self.opacity = opacity
        self.pointSize = pointSize
        self.colorMode = 0
        self.cameraPositionWorld = np.array([0, 0, 0])
        self.maxZ = 0.0 if maxZ is None else maxZ
        self.colorMapScale = 0.05 * colorMapScale

    def render(self, modelMatrix, viewMatrix, projectionMatrix):
        if self.pcProgram is None:
            assetDir = pathlib.Path(__file__).resolve().parent
            pcVert = (assetDir / "point_cloud.vert").read_text()
            # Poor man's GLSL templates
            replaceMap = {
                '{MAX_Z}': str(self.maxZ),
                '{COLOR_MAP_SCALE}': str(self.colorMapScale)
            }
            for k, v in replaceMap.items():
                pcVert = pcVert.replace(k, v)
            pcFrag = (assetDir / "point_cloud.frag").read_text()
            self.pcProgram = PointCloudProgram(createProgram(pcVert, pcFrag))

        if self.pointCloud is None: return

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_PROGRAM_POINT_SIZE)
        glEnable(GL_POINT_SPRITE)
        glEnable(GL_DEPTH_TEST)
        if self.opacity >= 1.0: glDepthMask(GL_TRUE)
        else: glDepthMask(GL_FALSE)

        glUseProgram(self.pcProgram.program)
        glUniformMatrix4fv(self.pcProgram.uniformModel, 1, GL_FALSE, modelMatrix.transpose())
        glUniformMatrix4fv(self.pcProgram.uniformView, 1, GL_FALSE, viewMatrix.transpose())
        glUniformMatrix4fv(self.pcProgram.uniformProjection, 1, GL_FALSE, projectionMatrix.transpose())
        glUniform3f(self.pcProgram.uniformCameraPositionWorld, self.cameraPositionWorld[0], self.cameraPositionWorld[1], self.cameraPositionWorld[2])

        glUniform1f(self.pcProgram.uniformPointSize, self.pointSize)
        glUniform1f(self.pcProgram.uniformOpacity, self.opacity)
        glUniform1i(self.pcProgram.uniformColorMode, self.colorMode)
        glUniform1i(self.pcProgram.uniformHasColor, 0 if self.pointCloud.colors is None else 1)
        glUniform1i(self.pcProgram.uniformHasNormal, 0 if self.pointCloud.normals is None else 1)

        if self.updated:
            self.updated = False

            # Delete old VBOs
            if self.vboPosition is not None:
                glDeleteBuffers(1, [self.vboPosition])

            if self.vboColor is not None:
                glDeleteBuffers(1, [self.vboColor])

            if self.vboNormal is not None:
                glDeleteBuffers(1, [self.vboNormal])

            # Create new VBOs
            self.vboPosition = glGenBuffers(1)

            # Bind and populate the new VBOs with updated data
            glBindBuffer(GL_ARRAY_BUFFER, self.vboPosition)
            glBufferData(GL_ARRAY_BUFFER, np.array(self.pointCloud.vertices, dtype=np.float32), GL_STATIC_DRAW)

            if self.pointCloud.colors is not None:
                self.vboColor = glGenBuffers(1)
                glBindBuffer(GL_ARRAY_BUFFER, self.vboColor)
                glBufferData(GL_ARRAY_BUFFER, np.array(self.pointCloud.colors, dtype=np.float32), GL_STATIC_DRAW)

            if self.pointCloud.normals is not None:
                self.vboNormal = glGenBuffers(1)
                glBindBuffer(GL_ARRAY_BUFFER, self.vboNormal)
                glBufferData(GL_ARRAY_BUFFER, np.array(self.pointCloud.normals, dtype=np.float32), GL_STATIC_DRAW)

        # Draw point cloud
        glEnableVertexAttribArray(0)
        glBindBuffer(GL_ARRAY_BUFFER, self.vboPosition)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, None)

        if self.vboColor is not None:
            glEnableVertexAttribArray(1)
            glBindBuffer(GL_ARRAY_BUFFER, self.vboColor)
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, None)

        if self.vboNormal is not None:
            glEnableVertexAttribArray(2)
            glBindBuffer(GL_ARRAY_BUFFER, self.vboNormal)
            glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, None)

        glDrawArrays(GL_POINTS, 0, len(self.pointCloud.vertices) // COORDS_PER_VERTEX)

        glDisableVertexAttribArray(0)
        if self.vboColor is not None: glDisableVertexAttribArray(1)
        if self.vboNormal is not None: glDisableVertexAttribArray(2)

        glUseProgram(0)
        glDisable(GL_PROGRAM_POINT_SIZE)
        glDisable(GL_BLEND)
        glDisable(GL_DEPTH_TEST)

    def setPointCloud(self, pointCloud):
        self.updated = True
        self.pointCloud = pointCloud

    def setPointSize(self, size):
        self.pointSize = size

    def setOpacity(self, opacity):
        self.opacity = opacity

    def setColorMode(self, mode):
        self.colorMode = mode

    def setCameraPositionWorld(self, position):
        self.cameraPositionWorld = position
