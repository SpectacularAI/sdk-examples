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
from mapping_ar_renderers.point_cloud import PointCloudRenderer

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
        targetResolution = [int(s) for s in args.resolution.split("x")]
        scale = None
        # Must be initialized after pygame.
        meshRenderer = None
        pointCloudRenderer = None
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
                targetWidth = state.targetResolution[0]
                targetHeight = state.targetResolution[1]
                state.scale = min(targetWidth / width, targetHeight / height)
                adjustedResolution = [int(state.scale * width), int(state.scale * height)]
                init_display(adjustedResolution[0], adjustedResolution[1])
                state.meshRenderer = MeshRenderer()
                state.pointCloudRenderer = PointCloudRenderer()

            for event in pygame.event.get():
                if event.type == pygame.QUIT: state.shouldQuit = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q: state.shouldQuit = True
                    if event.key == pygame.K_x: state.pointCloudMode = not state.pointCloudMode
                if state.shouldQuit: return

            glPixelZoom(state.scale, state.scale);
            glDrawPixels(width, height, GL_RGB, GL_UNSIGNED_BYTE, img.data)

            if state.lastMappingOutput:
                if state.pointCloudMode:
                    if state.pointCloudRenderer:
                        state.pointCloudRenderer.setPointCloud(state.lastMappingOutput)
                        state.pointCloudRenderer.setPose(cameraPose)
                        state.pointCloudRenderer.render()
                else:
                    if state.meshRenderer:
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
    p.add_argument("--resolution", help="Window resolution.", default="1920x1080")
    return p.parse_args()

if __name__ == '__main__':
    main(parseArgs())
