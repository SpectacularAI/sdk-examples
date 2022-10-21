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

POINT_CLOUD_STRIDE = 2
KEYFRAME_GROUP_SIZE = 2

def drawMesh(currentCameraPose, mapperOutput):
    if mapperOutput.mesh is None:
        print("No mesh to draw.")
        return

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    # glEnable(GL_BLEND);
    # glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    # TODO Draw

    # glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    print("ok")

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

    shouldQuit = False
    lastMappingOutput = None
    displayInitialized = False
    pointCloudMode = args.pointCloud

    def onVioOutput(vioOutput, frameSet):
        nonlocal lastMappingOutput
        nonlocal displayInitialized
        nonlocal pointCloudMode
        nonlocal shouldQuit

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

            if not displayInitialized:
                displayInitialized = True
                init_display(width, height)

            shouldQuit = False
            for event in pygame.event.get():
                if event.type == pygame.QUIT: shouldQuit = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q: shouldQuit = True
                    if event.key == pygame.K_x: pointCloudMode = not pointCloudMode
                if shouldQuit: return

            glDrawPixels(width, height, GL_RGB, GL_UNSIGNED_BYTE, img.data)
            if lastMappingOutput:
                if pointCloudMode:
                    drawPointCloud(cameraPose, lastMappingOutput)
                else:
                    drawMesh(cameraPose, lastMappingOutput)
            pygame.display.flip()

    def onMappingOutput(mapperOutput):
        nonlocal lastMappingOutput
        lastMappingOutput = mapperOutput

    if args.dataFolder:
        replay = spectacularAI.Replay(args.dataFolder, onMappingOutput, configuration=configInternal)
        replay.setExtendedOutputCallback(onVioOutput)
        replay.startReplay()
        while not shouldQuit:
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
