"""
Draw Mapping API outputs on video to create an AR visualization.
"""

import os
import time

import numpy as np

from OpenGL.GL import * # all prefixed with gl so OK to import *
# OpenGL.ERROR_CHECKING = False
# OpenGL.ERROR_LOGGING = False

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

import spectacularAI
import depthai

from mixed_reality import init_display, make_pipelines
from mapping_ar_renderers.mesh import MeshRenderer

POINT_CLOUD_STRIDE = 2
KEYFRAME_GROUP_SIZE = 2

def drawPointCloud(currentCameraPose, mapperOutput):
    glLoadIdentity()
    near, far = 0.01, 100.0 # clip
    glMultMatrixd(currentCameraPose.camera.getProjectionMatrixOpenGL(near, far).transpose())
    glMultMatrixd(currentCameraPose.getWorldToCameraMatrix().transpose())
    glPointSize(3)
    glColor3f(1, 0, 0)

    keyFrameIds = []
    for keyFrameId in mapperOutput.map.keyFrames:
        keyFrameIds.append(keyFrameId)
    keyFrameIds.sort(reverse=True)

    for i, keyFrameId in enumerate(keyFrameIds):
        if i >= KEYFRAME_GROUP_SIZE: return
        keyFrame = mapperOutput.map.keyFrames[keyFrameId]
        cameraPose = keyFrame.frameSet.primaryFrame.cameraPose

        glPushMatrix();
        glMultMatrixd(cameraPose.getCameraToWorldMatrix().transpose())
        if keyFrame.pointCloud:
            # TODO Use array-based geometry, per-vertex operations are very slow in PyOpenGL.
            glBegin(GL_POINTS)
            for i, p in enumerate(keyFrame.pointCloud.getPositionData()):
                if i % POINT_CLOUD_STRIDE != 0: continue
                glVertex3fv(p)
            glEnd()
        glPopMatrix();

def main(args):
    configInternal = {
        # "delayFrames": 0,
        # "noWaitingForResults": "true",
        # "maxSlamResultQueueSize": "2",

        "computeStereoPointCloud": "true",
        "pointCloudNormalsEnabled": "true",
        "computeDenseStereoDepth": "true",
        "recEnabled": "true",
    }
    if args.dataFolder and args.useRectification:
        configInternal["useRectification"] = "true"
    else:
        configInternal["alreadyRectified"] = "true"

    class State:
        shouldQuit = False
        lastMappingOutput = None
        displayInitialized = False
        pointCloudMode = args.pointCloud
        meshRenderer = None # Must be initialized after pygame.
    state = State()

    def onVioOutput(vioOutput, frameSet):
        nonlocal state

        cameraPose = vioOutput.getCameraPose(0)
        camToWorld = cameraPose.getCameraToWorldMatrix()
        for frame in frameSet:
            if not frame.image: continue
            if not frame.index == 0: continue # NOTE "primary frame"

            img = frame.image.toArray()
            # Flip the image upside down for OpenGL
            img = np.ascontiguousarray(np.flipud(img))
            width = img.shape[1]
            height = img.shape[0]

            if not state.displayInitialized:
                state.displayInitialized = True
                init_display(width, height)
                state.meshRenderer = MeshRenderer()

            for event in pygame.event.get():
                if event.type == pygame.QUIT: state.shouldQuit = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q: state.shouldQuit = True
                    if event.key == pygame.K_x: state.pointCloudMode = not state.pointCloudMode
                if state.shouldQuit: return

            glDrawPixels(width, height, GL_RGB, GL_UNSIGNED_BYTE, img.data)
            if state.lastMappingOutput:
                if state.pointCloudMode:
                    drawPointCloud(cameraPose, state.lastMappingOutput)
                else:
                    # TODO Set mesh onMappingOutput, but take care of synchronization.
                    state.meshRenderer.setMesh(state.lastMappingOutput.mesh)
                    state.meshRenderer.setPose(cameraPose)
                    state.meshRenderer.render()
            pygame.display.flip()

    def onMappingOutput(mapperOutput):
        nonlocal state
        state.lastMappingOutput = mapperOutput

    if args.dataFolder:
        replay = spectacularAI.Replay(args.dataFolder, onMappingOutput, configuration=configInternal)
        replay.setExtendedOutputCallback(onVioOutput)
        replay.startReplay()
        while not state.shouldQuit:
            time.sleep(0.05)
        replay.close()
        pygame.quit()
    else:
        def captureLoop():
            pipeline, vio_pipeline = make_pipelines(None, configInternal, onMappingOutput)
            with depthai.Device(pipeline) as device, \
                vio_pipeline.startSession(device) as vio_session:
                if args.ir_dot_brightness > 0:
                    device.setIrLaserDotProjectorBrightness(args.ir_dot_brightness)
                # TODO Not implemented.

def parseArgs():
    import argparse
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("--dataFolder", help="Instead of running live mapping session, replay session from this folder")
    p.add_argument("--use_rgb", help="Use OAK-D RGB camera", action="store_true")
    p.add_argument('--ir_dot_brightness', help='OAK-D Pro (W) IR laser projector brightness (mA), 0 - 1200', type=float, default=0)
    p.add_argument("--useRectification", help="--dataFolder option can also be used with some non-OAK-D recordings, but this parameter must be set if the videos inputs are not rectified.", action="store_true")
    p.add_argument("--pointCloud", help="Show point cloud instead of mesh.", action="store_true")
    return p.parse_args()

if __name__ == '__main__':
    main(parseArgs())
