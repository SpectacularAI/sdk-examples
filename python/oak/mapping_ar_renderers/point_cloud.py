import pathlib

import numpy as np

from OpenGL.GL import *

from .util import *

POINT_CLOUD_STRIDE = 2

class PointCloudProgram:
    def __init__(self, program):
        assert(program)
        self.program = program
        self.attributeVertex = glGetAttribLocation(self.program, "a_Position")
        self.uniformModelViewProjection = glGetUniformLocation(self.program, "u_ModelViewProjection")

class PointCloudRenderer:
    modelViewProjection = None
    pcProgram = None

    # 1d arrays of GLfloat.
    vertexData = np.array([])

    def __init__(self):
        assetDir = pathlib.Path(__file__).resolve().parent
        pcVert = (assetDir / "point_cloud.vert").read_text()
        pcFrag = (assetDir / "point_cloud.frag").read_text()
        self.pcProgram = PointCloudProgram(createProgram(pcVert, pcFrag))

    def render(self):
        if self.modelViewProjection is None: return
        if self.vertexData.shape[0] == 0: return

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_PROGRAM_POINT_SIZE)
        glEnable(GL_POINT_SPRITE) # TODO Check what this is, seems necessary.

        glUseProgram(self.pcProgram.program)
        glUniformMatrix4fv(self.pcProgram.uniformModelViewProjection, 1, GL_FALSE, self.modelViewProjection.transpose())

        coordsPerVertex = 3
        glEnableVertexAttribArray(self.pcProgram.attributeVertex)
        glVertexAttribPointer(self.pcProgram.attributeVertex, coordsPerVertex,
            GL_FLOAT, GL_FALSE, 0, self.vertexData)

        nVertices = self.vertexData.shape[0] // coordsPerVertex
        glDrawArrays(GL_POINTS, 0, nVertices)

        glDisableVertexAttribArray(self.pcProgram.attributeVertex)
        glUseProgram(0)

        glDisable(GL_PROGRAM_POINT_SIZE)
        glDisable(GL_BLEND)

    def setPointCloud(self, mapperOutput):
        # TODO Remove. Take just the current map points, maintain them with age.
        keyFrameIds = []
        for keyFrameId in mapperOutput.map.keyFrames:
            keyFrameIds.append(keyFrameId)
        keyFrameIds.sort(reverse=True)

        def affineTransform(M, p):
            return M[:3, :3] @ p + M[:3, 3]

        for i, keyFrameId in enumerate(keyFrameIds):
            # TODO This loop needs to append rather than replace vertexData if this
            # size is increased beyond 1.
            KEYFRAME_GROUP_SIZE = 1
            if i >= KEYFRAME_GROUP_SIZE: return

            keyFrame = mapperOutput.map.keyFrames[keyFrameId]
            if not keyFrame.pointCloud: continue
            cameraPose = keyFrame.frameSet.primaryFrame.cameraPose
            cToW = cameraPose.getCameraToWorldMatrix()
            points = keyFrame.pointCloud.getPositionData()
            n = points.shape[0]
            self.vertexData = np.zeros(3 * n)
            for i in range(n):
                self.vertexData[(3 * i):(3 * i + 3)] = affineTransform(cToW, points[i, :])

    def setPose(self, cameraPose):
        near, far = 0.01, 100.0
        projection = cameraPose.camera.getProjectionMatrixOpenGL(near, far)
        modelView = cameraPose.getWorldToCameraMatrix()
        self.modelViewProjection = projection @ modelView
