"""
Draw Mapping API outputs on video to create an AR visualization.

Requirements:
    pip install pygame PyOpenGL
And optionally to improve performance:
    pip install PyOpenGL_accelerate

For OAK-D live use:
    * Plug in OAK-D and run `python3 mapping_ar.py`.

For OAK-D replays:
    * Plug in OAK-D.
    * `python vio_record.py --slam --output recording`
    * `python mapping_ar.py --dataFolder recording`

For non-OAK-D replays, you may need to set more options, for example:
    * `python mapping_ar.py --dataFolder recording --useRectification --cameraInd 0`
"""

import os
import time

import numpy as np

from OpenGL.GL import * # all prefixed with gl so OK to import *

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

import spectacularAI
import depthai

from mixed_reality import init_display, make_pipelines
from mapping_ar_renderers.mesh import MeshRenderer
from mapping_ar_renderers.point_cloud import PointCloudRenderer
from mapping_ar_renderers.util import loadObjToMesh

class State:
    shouldQuit = False
    currentMapperOutput = None
    lastMapperOutput = None
    displayInitialized = False
    pointCloudMode = None
    targetResolution = None
    scale = None
    # Must be initialized after pygame.
    meshRenderer = None
    pointCloudRenderer = None
    mesh = None

def handleVioOutput(state, cameraPose, t, img, width, height):
    if not state.displayInitialized:
        state.displayInitialized = True
        targetWidth = state.targetResolution[0]
        targetHeight = state.targetResolution[1]
        state.scale = min(targetWidth / width, targetHeight / height)
        adjustedResolution = [int(state.scale * width), int(state.scale * height)]
        init_display(adjustedResolution[0], adjustedResolution[1])
        state.meshRenderer = MeshRenderer()
        state.pointCloudRenderer = PointCloudRenderer()
        if state.mesh:
            state.meshRenderer.setMesh(state.mesh)

    for event in pygame.event.get():
        if event.type == pygame.QUIT: state.shouldQuit = True
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q: state.shouldQuit = True
            if event.key == pygame.K_x: state.pointCloudMode = not state.pointCloudMode
            if event.key == pygame.K_m:
                if state.meshRenderer: state.meshRenderer.nextMode()
                if state.pointCloudRenderer: state.pointCloudRenderer.nextMode()
            if event.key == pygame.K_s: time.sleep(10) # TODO Remove.

        if state.shouldQuit: return

    glPixelZoom(state.scale, state.scale);
    glDrawPixels(width, height, GL_RGB, GL_UNSIGNED_BYTE, img.data)

    if state.pointCloudMode:
        if state.currentMapperOutput and state.pointCloudRenderer:
            if state.currentMapperOutput is not state.lastMapperOutput:
                state.pointCloudRenderer.setPointCloud(state.currentMapperOutput, t)
            state.pointCloudRenderer.setPose(cameraPose, t)
            state.pointCloudRenderer.render()
    else:
        if (state.currentMapperOutput or state.mesh) and state.meshRenderer:
            if not state.mesh and state.currentMapperOutput is not state.lastMapperOutput:
                state.meshRenderer.setMesh(state.currentMapperOutput.mesh)
            state.meshRenderer.setPose(cameraPose)
            state.meshRenderer.render()

    if state.currentMapperOutput:
        state.lastMapperOutput = state.currentMapperOutput
    pygame.display.flip()

def oakdLoop(args, state, device, vioSession):
    img_queue = device.getOutputQueue(name="cam_out", maxSize=4, blocking=False)
    # Buffer for frames: show together with the corresponding VIO output
    frames = {}
    frame_number = 1

    while True:
        if img_queue.has():
            img = img_queue.get()
            img_time = img.getTimestamp().total_seconds()
            frames[frame_number] = img
            vioSession.addTrigger(img_time, frame_number)
            frame_number += 1
        elif vioSession.hasOutput():
            vioOutput = vioSession.getOutput()
            if vioOutput.tag > 0:
                img = frames.get(vioOutput.tag)
                cameraPose = vioSession.getRgbCameraPose(vioOutput)
                time = vioOutput.pose.time
                handleVioOutput(state, cameraPose, time, img.getRaw().data, img.getWidth(), img.getHeight())
                # Discard old tags.
                frames = { tag: v for tag, v in frames.items() if tag > vioOutput.tag }
        else:
            pygame.time.wait(1)
        if state.shouldQuit:
            return

def main(args):
    print("Control using the keyboard:")
    print("* Q: Quit")
    print("* X: Change between mesh and point cloud visualizations")
    print("* M: Cycle mesh visualization options.")
    print("------\n")

    if args.mapLoadPath:
        configInternal = {
            "mapLoadPath": args.mapLoadPath,
            "useSlam": "true",
        }
    else:
        configInternal = {
            "computeStereoPointCloud": "true",
            "pointCloudNormalsEnabled": "true",
            "computeDenseStereoDepth": "true",
            "computeDenseStereoDepthKeyFramesOnly": "true",
            "recEnabled": "true",
        }
    if args.useRectification:
        configInternal["useRectification"] = "true"
    else:
        configInternal["alreadyRectified"] = "true"

    state = State()
    state.pointCloudMode = args.pointCloud
    state.targetResolution = [int(s) for s in args.resolution.split("x")]
    if args.objLoadPath:
        state.mesh = loadObjToMesh(args.objLoadPath)

    def replayOnVioOutput(vioOutput, frameSet):
        nonlocal state
        for frame in frameSet:
            if not frame.image: continue
            if not frame.index == args.cameraInd: continue
            img = frame.image.toArray()
            img = np.ascontiguousarray(np.flipud(img)) # Flip the image upside down for OpenGL.
            width = img.shape[1]
            height = img.shape[0]
            time = vioOutput.pose.time
            handleVioOutput(state, frame.cameraPose, time, img, width, height)

    def onMappingOutput(mapperOutput):
        nonlocal state
        state.currentMapperOutput = mapperOutput

    if args.mapLoadPath:
        onMappingOutput = None # Appending to existing map is not currently supported.
    else:
        def onMappingOutput(mapperOutput):
            nonlocal state
            state.currentMapperOutput = mapperOutput

    if args.dataFolder:
        replay = spectacularAI.Replay(args.dataFolder, onMappingOutput, configuration=configInternal)
        replay.setExtendedOutputCallback(replayOnVioOutput)
        replay.startReplay()
        while not state.shouldQuit:
            time.sleep(0.05)
        replay.close()
        pygame.quit()
    else:
        pipeline, vio_pipeline = make_pipelines(args.mapLoadPath, configInternal, onMappingOutput)
        with depthai.Device(pipeline) as device, \
            vio_pipeline.startSession(device) as vioSession:
            if args.ir_dot_brightness > 0:
                device.setIrLaserDotProjectorBrightness(args.ir_dot_brightness)
            oakdLoop(args, state, device, vioSession)

def parseArgs():
    import argparse
    p = argparse.ArgumentParser(__doc__)
    # Generic parameters.
    p.add_argument("--resolution", help="Window resolution.", default="1920x1080")
    p.add_argument("--pointCloud", help="Start in the point cloud mode.", action="store_true")
    p.add_argument("--dataFolder", help="Instead of running live mapping session, replay session from this folder")
    # OAK-D parameters.
    p.add_argument('--ir_dot_brightness', help='OAK-D Pro (W) IR laser projector brightness (mA), 0 - 1200', type=float, default=0)
    # Parameters for non-OAK-D recordings.
    p.add_argument("--useRectification", help="--dataFolder option can also be used with some non-OAK-D recordings, but this parameter must be set if the videos inputs are not rectified.", action="store_true")
    p.add_argument('--objLoadPath', help="Load scene as .obj", default=None)
    p.add_argument('--cameraInd', help="Which camera to use. Typically 0=left, 1=right, 2=auxiliary/RGB (OAK-D default)", type=int, default=2)
    p.add_argument("--mapLoadPath", help="SLAM map path", default=None)
    return p.parse_args()

if __name__ == '__main__':
    main(parseArgs())
