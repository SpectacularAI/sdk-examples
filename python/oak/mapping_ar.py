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
import subprocess
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
    args = None
    shouldQuit = False
    recordPipe = None
    currentMapperOutput = None
    lastMapperOutput = None
    displayInitialized = False
    targetResolution = None
    adjustedResolution = None
    scale = None
    # Must be initialized after pygame.
    meshRenderer = None
    pointCloudRenderer = None
    mesh = None

def updateRenderer(state, cameraPose, t):
    if state.args.pointCloud:
        if not state.pointCloudRenderer: return
        if not state.currentMapperOutput: return
        if state.currentMapperOutput is not state.lastMapperOutput:
            state.pointCloudRenderer.setPointCloud(state.currentMapperOutput, t)
        state.pointCloudRenderer.setPose(cameraPose, t)
        state.pointCloudRenderer.render()
    else:
        def renderMesh():
            state.meshRenderer.setPose(cameraPose)
            state.meshRenderer.render()

        def updateMesh():
            state.meshRenderer.setMesh(state.currentMapperOutput.mesh)

        if not state.meshRenderer: return
        if not state.currentMapperOutput: return
        if state.mesh: return renderMesh()

        if not state.lastMapperOutput: updateMesh()
        elif state.currentMapperOutput.mesh is not state.lastMapperOutput.mesh: updateMesh()
        return renderMesh()

def handleVioOutput(state, cameraPose, t, img, width, height):
    if state.shouldQuit:
        return

    if not state.displayInitialized:
        state.displayInitialized = True
        targetWidth = state.targetResolution[0]
        targetHeight = state.targetResolution[1]
        state.scale = min(targetWidth / width, targetHeight / height)
        state.adjustedResolution = [int(state.scale * width), int(state.scale * height)]
        init_display(state.adjustedResolution[0], state.adjustedResolution[1])
        state.meshRenderer = MeshRenderer()
        state.pointCloudRenderer = PointCloudRenderer(state.args.pointCloudDensity)
        if state.mesh:
            state.meshRenderer.setMesh(state.mesh)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            state.shouldQuit = True
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q: state.shouldQuit = True
            if event.key == pygame.K_x: state.args.pointCloud = not state.args.pointCloud
            if event.key == pygame.K_m:
                if state.meshRenderer: state.meshRenderer.nextMode()
                if state.pointCloudRenderer: state.pointCloudRenderer.nextMode()
        if state.shouldQuit:
            pygame.quit()
            return

    glPixelZoom(state.scale, state.scale)
    glDrawPixels(width, height, GL_RGB, GL_UNSIGNED_BYTE, img.data)

    updateRenderer(state, cameraPose, t)

    if state.currentMapperOutput:
        state.lastMapperOutput = state.currentMapperOutput

    if state.args.recordPath:
        try: os.makedirs(os.path.dirname(state.args.recordPath))
        except: pass

        if os.name == 'nt':
            ffmpegStdErrToNull = "2>NUL"
        else:
            ffmpegStdErrToNull = "2>/dev/null"

        r = state.adjustedResolution
        if state.recordPipe is None:
            cmd = "ffmpeg -y -f rawvideo -vcodec rawvideo -pix_fmt rgb24 -s {}x{} -i - -an -pix_fmt yuv420p -c:v libx264 -vf vflip -crf 17 \"{}\" {}".format(
                r[0], r[1], state.args.recordPath, ffmpegStdErrToNull)
            state.recordPipe = subprocess.Popen(
                cmd, stdin=subprocess.PIPE, shell=True)
        buffer = glReadPixels(0, 0, r[0], r[1], GL_RGB, GL_UNSIGNED_BYTE)
        state.recordPipe.stdin.write(buffer)

    pygame.display.flip()

def oakdLoop(state, device, vioSession):
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
    print("* M: Cycle through visualization options.")
    print("------\n")

    if args.mapLoadPath:
        configInternal = {
            "mapLoadPath": args.mapLoadPath,
            "useSlam": "true",
            "fixedMap": "true",
        }
    else:
        configInternal = {
            "computeStereoPointCloud": "true",
            "pointCloudNormalsEnabled": "true",
            "computeDenseStereoDepth": "true",
            "computeDenseStereoDepthKeyFramesOnly": "true",
            "recEnabled": "true",
            "recCellSize": "0.02",
            "useSlam": "true",
        }
    if args.useRectification:
        configInternal["useRectification"] = "true"
    else:
        configInternal["alreadyRectified"] = "true"
    configInternal["stereoPointCloudMaxDepth"] = str(args.depth)
    configInternal["recMaxDistance"] = str(args.depth)

    state = State()
    state.args = args
    state.targetResolution = [int(s) for s in args.resolution.split("x")]
    if args.objLoadPath:
        position = [float(s) for s in args.objPosition.split(",")]
        state.mesh = loadObjToMesh(args.objLoadPath, position)

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

    if args.mapLoadPath:
        onMappingOutput = None # Appending to existing map is not currently supported.
    else:
        def onMappingOutput(mapperOutput):
            nonlocal state
            state.currentMapperOutput = mapperOutput
            if mapperOutput.finalMap:
                # TODO: call pygame.quit() BUT it has to be called from same thread as init_display(...)
                state.shouldQuit = True

    if args.dataFolder:
        replay = spectacularAI.Replay(args.dataFolder, onMappingOutput, configuration=configInternal)
        replay.setExtendedOutputCallback(replayOnVioOutput)
        replay.startReplay()
        while not state.shouldQuit:
            time.sleep(0.05)
        print("Quitting...")
        replay.close()
    else:
        config = spectacularAI.depthai.Configuration()
        config.internalParameters = configInternal
        if args.noFeatureTracker: config.useFeatureTracker = False
        if args.mapLoadPath is not None:
            config.mapLoadPath = args.mapLoadPath
            config.useSlam = True

        pipeline, vio_pipeline = make_pipelines(config, onMappingOutput)
        with depthai.Device(pipeline) as device, \
            vio_pipeline.startSession(device) as vioSession:
            if args.irBrightness > 0:
                device.setIrLaserDotProjectorBrightness(args.irBrightness)
            oakdLoop(state, device, vioSession)
            print("Quitting...")

def parseArgs():
    import argparse
    p = argparse.ArgumentParser(__doc__)
    # Generic parameters.
    p.add_argument("--resolution", help="Window resolution.", default="1920x1080")
    p.add_argument("--pointCloud", help="Start in the point cloud mode.", action="store_true")
    p.add_argument("--pointCloudDensity", help="Fraction of points to show.", default=0.2, type=float)
    p.add_argument("--dataFolder", help="Instead of running live mapping session, replay session from this folder")
    p.add_argument("--recordPath", help="Record the window to video file given by path.")
    p.add_argument("--depth", help="In meters, the max distance to detect points and construct mesh. Lower values may improve positioning accuracy.", default=4, type=float)
    # OAK-D parameters.
    p.add_argument('--irBrightness', help='OAK-D Pro (W) IR laser projector brightness (mA), 0 - 1200. Enabling may improve depth tracking.', type=float, default=0)
    p.add_argument('--noFeatureTracker', help="On OAK-D, use stereo images rather than accelerated features + depth.", action="store_true")
    # Parameters for non-OAK-D recordings.
    p.add_argument("--useRectification", help="--dataFolder option can also be used with some non-OAK-D recordings, but this parameter must be set if the videos inputs are not rectified.", action="store_true")
    p.add_argument('--objLoadPath', help="Load scene as .obj", default=None)
    p.add_argument('--objPosition', help="Set position of .obj mesh", default="0,0,0")
    p.add_argument('--cameraInd', help="Which camera to use. Typically 0=left, 1=right, 2=auxiliary/RGB (OAK-D default)", type=int, default=2)
    p.add_argument("--mapLoadPath", help="SLAM map path", default=None)
    return p.parse_args()

if __name__ == '__main__':
    main(parseArgs())
