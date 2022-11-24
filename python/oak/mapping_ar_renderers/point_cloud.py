import pathlib
import random

import numpy as np

from OpenGL.GL import *

from .util import *

MAX_AGE_SECONDS = 3

COORDS_PER_VERTEX = 3
COORDS_PER_EFFECT = 3

class PointCloud:
    vertexData = np.array([])
    effectData = np.array([])
    time = None

class PointCloudProgram:
    def __init__(self, program):
        assert(program)
        self.program = program
        self.attributeVertex = glGetAttribLocation(self.program, "a_Position")
        self.attributeEffect = glGetAttribLocation(self.program, "a_Effect")
        self.uniformModelView = glGetUniformLocation(self.program, "u_ModelView")
        self.uniformProjection = glGetUniformLocation(self.program, "u_Projection")

class PointCloudRenderer:
    # Value greater than zero enables at index:
    # 0) Show camera view.
    OPTIONS = [
        [1],
        [0],
    ]

    selectedOption = 0
    modelView = None
    projection = None
    pcProgram = None
    density = None
    pointClouds = []

    # 1d arrays of GLfloat.
    vertexData = np.array([])
    effectData = np.array([])

    def __init__(self, density):
        self.density = density

        assetDir = pathlib.Path(__file__).resolve().parent
        pcVert = (assetDir / "point_cloud.vert").read_text()
        pcFrag = (assetDir / "point_cloud.frag").read_text()
        self.pcProgram = PointCloudProgram(createProgram(pcVert, pcFrag))

    def render(self):
        if self.modelView is None: return
        if self.vertexData.shape[0] == 0: return

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_PROGRAM_POINT_SIZE)
        glEnable(GL_POINT_SPRITE)

        op = PointCloudRenderer.OPTIONS[self.selectedOption]
        if op[0] == 0.0: glClear(GL_COLOR_BUFFER_BIT)

        glUseProgram(self.pcProgram.program)
        glUniformMatrix4fv(self.pcProgram.uniformModelView, 1, GL_FALSE, self.modelView.transpose())
        glUniformMatrix4fv(self.pcProgram.uniformProjection, 1, GL_FALSE, self.projection.transpose())

        glEnableVertexAttribArray(self.pcProgram.attributeVertex)
        glVertexAttribPointer(self.pcProgram.attributeVertex, COORDS_PER_VERTEX,
            GL_FLOAT, GL_FALSE, 0, self.vertexData)

        glEnableVertexAttribArray(self.pcProgram.attributeEffect)
        glVertexAttribPointer(self.pcProgram.attributeEffect, COORDS_PER_EFFECT,
            GL_FLOAT, GL_FALSE, 0, self.effectData)

        nVertices = self.vertexData.shape[0] // COORDS_PER_VERTEX
        glDrawArrays(GL_POINTS, 0, nVertices)

        glDisableVertexAttribArray(self.pcProgram.attributeEffect)
        glDisableVertexAttribArray(self.pcProgram.attributeVertex)
        glUseProgram(0)

        glDisable(GL_PROGRAM_POINT_SIZE)
        glDisable(GL_BLEND)

    def setPointCloud(self, mapperOutput, time):
        keyFrameId = max(mapperOutput.map.keyFrames.keys())
        if keyFrameId is None: return

        def affineTransform(M, p):
            return M[:3, :3] @ p + M[:3, 3]

        keyFrame = mapperOutput.map.keyFrames[keyFrameId]
        if not keyFrame.pointCloud: return
        cameraPose = keyFrame.frameSet.primaryFrame.cameraPose
        cToW = cameraPose.getCameraToWorldMatrix()
        points = keyFrame.pointCloud.getPositionData()

        selected = []
        for i in range(points.shape[0]):
            if random.random() < self.density: selected.append(i)
        n = len(selected)

        cv = COORDS_PER_VERTEX
        pointCloud = PointCloud()
        pointCloud.vertexData = np.zeros(cv * n)
        pointCloud.effectData = np.zeros(COORDS_PER_EFFECT * n)
        pointCloud.time = time
        j = 0
        for i in selected:
            pointCloud.vertexData[(cv * j):(cv * j + cv)] = affineTransform(cToW, points[i, :])
            j += 1
        self.pointClouds.append(pointCloud)

    def setPose(self, cameraPose, time):
        near, far = 0.01, 100.0
        self.projection = cameraPose.camera.getProjectionMatrixOpenGL(near, far)
        self.modelView = cameraPose.getWorldToCameraMatrix()

        while len(self.pointClouds) > 0 and time - self.pointClouds[0].time > MAX_AGE_SECONDS:
            self.pointClouds.pop(0)

        nv = sum([p.vertexData.shape[0] for p in self.pointClouds])
        ne = sum([p.effectData.shape[0] for p in self.pointClouds])
        self.vertexData = np.zeros(nv)
        self.effectData = np.zeros(ne)
        iv = 0
        ie = 0
        TIME_DISAPPEAR = 0.7
        for p in self.pointClouds:
            self.vertexData[iv:(iv + p.vertexData.shape[0])] = p.vertexData
            iv += p.vertexData.shape[0]

            timeRelative = (time - p.time) / MAX_AGE_SECONDS
            for j in range(p.effectData.shape[0] // COORDS_PER_EFFECT):
                p.effectData[COORDS_PER_EFFECT * j + 2] = timeRelative
                if timeRelative > TIME_DISAPPEAR:
                    p.effectData[COORDS_PER_EFFECT * j + 0] += 0.002 * random.random()
                    p.effectData[COORDS_PER_EFFECT * j + 1] += 0.006 * random.random()
            self.effectData[ie:(ie + p.effectData.shape[0])] = p.effectData
            ie += p.effectData.shape[0]
        assert(iv == nv)
        assert(ie == ne)

    def nextMode(self):
        self.selectedOption = (self.selectedOption + 1) % len(PointCloudRenderer.OPTIONS)
