import os
import numpy as np
import time

from threading import Lock
from enum import Enum
from OpenGL.GL import * # all prefixed with gl so OK to import *

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

from .visualizer_renderers.mesh import MeshRenderer
from .visualizer_renderers.util import lookAt, getOrthographicProjectionMatrixOpenGL
from .visualizer_renderers.renderers import *

class CameraMode(Enum):
    AR = 0
    THIRD_PERSON = 1
    TOP_VIEW = 2

    def next(self):
        return CameraMode((self.value + 1) % 3)

class ColorMode(Enum):
    ORIGINAL = 0
    COORDINATE_X = 1
    COORDINATE_Y = 2
    COORDINATE_Z = 3
    DEPTH = 4
    NORMAL = 5

class CameraSmooth:
    def __init__(self):
        self.alpha = np.array([0.01, 0.01, 0.001])
        self.prevLookAtEye = None
        self.prevLookAtTarget = None

    def compute(self, eye, target, paused):
        if self.prevLookAtEye is not None:
            if paused: return self.prevLookAtEye, self.prevLookAtTarget
            eyeSmooth = self.alpha * eye + (1.0 - self.alpha) * self.prevLookAtEye
            targetSmooth = self.alpha * target + (1.0 - self.alpha) * self.prevLookAtTarget
        else:
            eyeSmooth = eye
            targetSmooth = target

        self.prevLookAtEye = eyeSmooth
        self.prevLookAtTarget = targetSmooth

        return eyeSmooth, targetSmooth

    def reset(self):
        self.prevLookAtEye = None
        self.prevLookAtTarget = None

class VisualizerArgs:
    # Window
    resolution = "1280x720" # Window resolution
    fullScreen = False # Full screen mode
    visualizationScale = 10.0 # Generic scale of visualizations. Affects color maps, camera size, etc.
    backGroundColor = [1, 1, 1] # Background color RGB color (0-1).
    keepOpenAfterFinalMap = False # If false, window is automatically closed on final mapper output

    # Camera
    cameraNear = 0.01 # Camera near plane (m)
    cameraFar = 100.0 # Camera far plane (m)
    cameraMode = CameraMode.THIRD_PERSON # Camera mode (options: AR, 3rd person, 2D). Note: AR mode should have 'useRectification: True'
    cameraSmooth = True # Enable camera smoothing in 3rd person mode
    flip = False # Vertically flip image in AR mode

    # Initial state for visualization components
    showGrid = True
    showPoseTrail = True
    showCameraModel = True
    showCameraFrustum = True

    # SLAM map
    pointSize = 2.0
    pointOpacity = 1.0
    pointCloudVoxelSize = None # Voxel size (m) for downsampling point clouds
    pointCloudMaxHeight = None # Point cloud max height (m) in world coordinates. Visualization only
    skipPointsWithoutColor = False # Deletes points whose color is [0, 0, 0]. Visualization only
    keyFrameCount = None # None = show all. Visualization only
    colorMode = ColorMode.ORIGINAL # Point cloud shader mode
    showPointCloud = True # Show key frame point clouds
    showKeyFrames = True # Show key frame poses
    showMesh = False # Show SLAM map mesh. Note: requires 'recEnabled: True'

    # Pose trail
    poseTrailLength = None # Number of frames in pose trail (unlimited=None)

    # Grid
    gridRadius = 20 # Grid side length is 2*n
    gridCellLength = 1.0 # Length of single cell (m)
    gridOrigin = [0., 0., 0.] # Grid origin in world coordinates

    # Camera frustum
    frustumNear = 0.2 # Frustum near plane (m). Visualization only
    frustumFar = 20.0 # Frustum far plane (m). Visualization only

    # Recording
    recordPath = None # Record the window to video file given by path

class Recorder:
    def __init__(self, recordPath, resolution):
        import subprocess

        os.makedirs(os.path.dirname(recordPath), exist_ok=True)
        if os.name == 'nt':
            ffmpegStdErrToNull = "2>NUL"
        else:
            ffmpegStdErrToNull = "2>/dev/null"

        self.resolution = resolution
        cmd = "ffmpeg -y -f rawvideo -vcodec rawvideo -pix_fmt rgb24 -s {}x{} -i - -an -pix_fmt yuv420p -c:v libx264 -vf vflip -crf 17 \"{}\" {}".format(
            resolution[0], resolution[1], recordPath, ffmpegStdErrToNull)
        self.recordPipe = subprocess.Popen(cmd, stdin=subprocess.PIPE, shell=True)

    def recordFrame(self):
        buffer = glReadPixels(0, 0, self.resolution[0], self.resolution[1], GL_RGB, GL_UNSIGNED_BYTE)
        self.recordPipe.stdin.write(buffer)

class Visualizer:
    def __init__(self, args=VisualizerArgs()):
        self.args = args

        # State
        self.shouldQuit = False
        self.shouldPause = False
        self.displayInitialized = False
        self.outputQueue = []
        self.outputQueueMutex = Lock()

        # Window
        self.fullScreen = args.fullScreen
        self.targetResolution = [int(s) for s in args.resolution.split("x")]
        self.adjustedResolution = None
        self.scale = None
        self.aspectRatio = None

        # Camera
        self.cameraMode = self.args.cameraMode
        self.cameraSmooth = CameraSmooth() if args.cameraSmooth else None
        self.initialZoom = args.visualizationScale / 10.0
        self.zoom = self.initialZoom

        # Toggle visualization components
        self.showGrid = args.showGrid
        self.showPoseTrail = args.showPoseTrail
        self.showCameraModel = args.showCameraModel
        self.showCameraFrustum = args.showCameraFrustum

        # Renderers
        self.map = MapRenderer(
            pointSize=args.pointSize,
            pointOpacity=args.pointOpacity,
            colorMode=args.colorMode.value,
            voxelSize=args.pointCloudVoxelSize,
            keyFrameCount=args.keyFrameCount,
            maxZ=args.pointCloudMaxHeight,
            skipPointsWithoutColor=args.skipPointsWithoutColor,
            visualizationScale=args.visualizationScale,
            renderPointCloud=args.showPointCloud,
            renderKeyFrames=args.showKeyFrames,
            renderMesh=args.showMesh)
        self.poseTrail = PoseTrailRenderer(maxLength=args.poseTrailLength)
        self.grid = GridRenderer(radius=args.gridRadius, length=args.gridCellLength, origin=args.gridOrigin)
        self.cameraModelRenderer = MeshRenderer(createCameraModelMesh(scale=args.visualizationScale / 20.0))
        self.cameraFrustumRenderer = None # initialized later when camera projection matrix is available

        # Recording
        self.recorder = None

    def __initDisplay(self):
        from pygame.locals import DOUBLEBUF, OPENGL, FULLSCREEN, GL_MULTISAMPLEBUFFERS, GL_MULTISAMPLESAMPLES

        if self.adjustedResolution is None: return
        w = self.adjustedResolution[0]
        h = self.adjustedResolution[1]
        self.displayInitialized = True

        pygame.init()

        # Configure multi-sampling (anti-aliasing)
        pygame.display.gl_set_attribute(GL_MULTISAMPLEBUFFERS, 1)
        pygame.display.gl_set_attribute(GL_MULTISAMPLESAMPLES, 4)
        if self.fullScreen: pygame.display.set_mode((w, h), DOUBLEBUF | OPENGL | FULLSCREEN)
        else: pygame.display.set_mode((w, h), DOUBLEBUF | OPENGL)
        pygame.display.set_caption("Spectacular AI Visualizer")

        # Enable multi-sampling in OpenGL
        glEnable(GL_MULTISAMPLE)

    def __close(self):
        assert(self.shouldQuit)
        if self.displayInitialized:
            self.displayInitialized = False
            pygame.quit()

    def __render(self, cameraPose, width, height, image=None, colorFormat=None):
        import spectacularAI

        if not self.displayInitialized:
            targetWidth = self.targetResolution[0]
            targetHeight = self.targetResolution[1]
            self.scale = min(targetWidth / width, targetHeight / height)
            self.adjustedResolution = [int(self.scale * width), int(self.scale * height)]
            self.aspectRatio = targetWidth / targetHeight
            self.cameraFrustumRenderer = CameraFrustumRenderer(cameraPose.camera.getProjectionMatrixOpenGL(self.args.frustumNear, self.args.frustumFar))
            self.recorder = Recorder(self.args.recordPath, self.adjustedResolution) if self.args.recordPath else None
            self.__initDisplay()

        glPixelZoom(self.scale, self.scale)
        glClearColor(*self.args.backGroundColor, 1.0)
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)

        near, far = self.args.cameraNear, self.args.cameraFar
        if self.cameraMode == CameraMode.AR:
            if image is not None:
                # draw AR background
                glDrawPixels(width, height, GL_LUMINANCE if colorFormat == spectacularAI.ColorFormat.GRAY else GL_RGB, GL_UNSIGNED_BYTE, image.data)
            viewMatrix = cameraPose.getWorldToCameraMatrix()
            projectionMatrix = cameraPose.camera.getProjectionMatrixOpenGL(near, far)
            if self.args.flip: projectionMatrix = np.array([1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]) @ projectionMatrix
        elif self.cameraMode == CameraMode.THIRD_PERSON:
            # TODO: implement mouse controls
            cameraToWorld = cameraPose.getCameraToWorldMatrix()
            up = np.array([0.0, 0.0, 1.0])
            forward = cameraToWorld[0:3, 2]
            eye = cameraToWorld[0:3, 3] - 10.0 * forward + 5.0 * up
            target = cameraToWorld[0:3, 3]
            if self.cameraSmooth: eye, target = self.cameraSmooth.compute(eye, target, self.shouldPause)
            viewMatrix = lookAt(eye, target, up)
            projectionMatrix = cameraPose.camera.getProjectionMatrixOpenGL(near, far)
            projectionMatrix[0, 0] *= 1.0 / self.zoom
            projectionMatrix[1, 1] *= 1.0 / self.zoom
        elif self.cameraMode == CameraMode.TOP_VIEW:
            cameraToWorld = cameraPose.getCameraToWorldMatrix()
            eye = cameraToWorld[0:3, 3] + np.array([0, 0, 15])
            target = cameraToWorld[0:3, 3]
            up = np.array([-1.0, 0.0, 0.0])
            viewMatrix = lookAt(eye, target, up)
            left = -25.0 * self.zoom
            right = 25.0 * self.zoom
            bottom = -25.0 * self.zoom / self.aspectRatio # divide by aspect ratio to avoid strecthing (i.e. x and y directions have equal scale)
            top = 25.0 * self.zoom / self.aspectRatio
            projectionMatrix = getOrthographicProjectionMatrixOpenGL(left, right, bottom, top, -1000.0, 1000.0)

        self.map.render(cameraPose.getPosition(), viewMatrix, projectionMatrix)
        if self.showGrid: self.grid.render(viewMatrix, projectionMatrix)
        if self.showPoseTrail: self.poseTrail.render(viewMatrix, projectionMatrix)

        if self.cameraMode in [CameraMode.THIRD_PERSON, CameraMode.TOP_VIEW]:
            modelMatrix = cameraPose.getCameraToWorldMatrix()
            if self.showCameraModel: self.cameraModelRenderer.render(modelMatrix, viewMatrix, projectionMatrix)
            if self.showCameraFrustum: self.cameraFrustumRenderer.render(modelMatrix, viewMatrix, projectionMatrix, self.cameraMode is CameraMode.TOP_VIEW)

        if self.recorder: self.recorder.recordFrame()
        pygame.display.flip()

    def __processUserInput(self):
        if not self.displayInitialized: return
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.shouldQuit = True
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q: self.shouldQuit = True
                elif event.key == pygame.K_SPACE: self.shouldPause = not self.shouldPause
                elif event.key == pygame.K_c:
                    self.cameraMode = self.cameraMode.next()
                    self.zoom = self.initialZoom
                    if self.cameraSmooth: self.cameraSmooth.reset()
                elif event.key == pygame.K_PLUS:
                    self.map.setPointSize(np.clip(self.map.pointSize*1.05, 0.0, 10.0))
                elif event.key == pygame.K_MINUS:
                    self.map.setPointSize(np.clip(self.map.pointSize*0.95, 0.0, 10.0))
                elif event.key == pygame.K_0:
                    self.map.setColorMode(ColorMode.ORIGINAL.value)
                elif event.key == pygame.K_1:
                    self.map.setColorMode(ColorMode.COORDINATE_X.value)
                elif event.key == pygame.K_2:
                    self.map.setColorMode(ColorMode.COORDINATE_Y.value)
                elif event.key == pygame.K_3:
                    self.map.setColorMode(ColorMode.COORDINATE_Z.value)
                elif event.key == pygame.K_4:
                    self.map.setColorMode(ColorMode.DEPTH.value)
                elif event.key == pygame.K_5:
                    self.map.setColorMode(ColorMode.NORMAL.value)
                elif event.key == pygame.K_m:
                    self.map.setRenderMesh(not self.map.renderMesh)
                elif event.key == pygame.K_n:
                    self.map.setRenderPointCloud(not self.map.renderPointCloud)
                elif event.key == pygame.K_k:
                    self.map.setRenderKeyFrames(not self.map.renderKeyFrames)
                elif event.key == pygame.K_g:
                    self.showGrid = not self.showGrid
                elif event.key == pygame.K_p:
                    self.showPoseTrail = not self.showPoseTrail
                elif event.key == pygame.K_f:
                    from pygame.locals import DOUBLEBUF, OPENGL, FULLSCREEN
                    w, h = self.adjustedResolution[0], self.adjustedResolution[1]
                    self.fullScreen = not self.fullScreen
                    if self.fullScreen: pygame.display.set_mode((w, h), DOUBLEBUF | OPENGL | FULLSCREEN)
                    else: pygame.display.set_mode((w, h), DOUBLEBUF | OPENGL)
                elif event.key == pygame.K_h:
                    self.printHelp()
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4: # Mouse wheel up
                    self.zoom = min(10.0*self.initialZoom , self.zoom * 1.05)
                elif event.button == 5: # Mouse wheel down
                    self.zoom = max(0.1*self.initialZoom , self.zoom * 0.95)

    def onVioOutput(self, cameraPose, image=None, width=None, height=None, colorFormat=None):
        if self.shouldQuit: return

        if image is None:
            output = {
                "type": "vio",
                "cameraPose" : cameraPose,
                "image" : None,
                "width" : self.targetResolution[0],
                "height" : self.targetResolution[1],
                "colorFormat" : None
            }
        else:
            # Flip the image upside down for OpenGL.
            if not self.args.flip: image = np.ascontiguousarray(np.flipud(image))
            output = {
                "type" : "vio",
                "cameraPose" : cameraPose,
                "image" : image,
                "width" : width,
                "height" : height,
                "colorFormat" : colorFormat
            }

        if self.outputQueueMutex:
            self.outputQueue.append(output)

        # Blocks VIO until previous outputs have been processed
        while len(self.outputQueue) > 5:
            time.sleep(0.01)
            if self.shouldQuit: break

    def onMappingOutput(self, mapperOutput):
        if self.shouldQuit: return

        output = {
            "type" : "slam",
            "mapperOutput" : mapperOutput
        }

        if self.outputQueueMutex:
            self.outputQueue.append(output)

    def run(self):
        while not self.shouldQuit:
            self.__processUserInput()

            if self.shouldPause or len(self.outputQueue) == 0:
                time.sleep(0.01)
                continue

            # Process VIO & Mapping API outputs
            vioOutput = None
            while self.outputQueueMutex and len(self.outputQueue) > 0:
                output = self.outputQueue.pop(0)
                if output["type"] == "vio":
                    vioOutput = output
                    cameraPose = vioOutput["cameraPose"]
                    self.poseTrail.append(cameraPose.getPosition())
                    break
                elif output["type"] == "slam":
                    mapperOutput = output["mapperOutput"]
                    self.map.onMappingOutput(mapperOutput)
                    if mapperOutput.finalMap and not self.args.keepOpenAfterFinalMap:
                        self.shouldQuit = True
                else:
                    print("Unknown output type: {}".format(output["type"]))

            if vioOutput:
                self.__render(
                    vioOutput["cameraPose"],
                    vioOutput["width"],
                    vioOutput["height"],
                    vioOutput["image"],
                    vioOutput["colorFormat"])

        self.__close()

    def printHelp(self):
        print("Control using the keyboard:")
        print("* Q: Quit")
        print("* SPACE: Pause")
        print("* SCROLL: Zoom")
        print("* F: Toggle fullscreen")
        print("* C: Cycle through visualization options (AR/THIRD PERSON/2D)")
        print("* M: Toggle SLAM map mesh (TODO)")
        print("* N: Toggle SLAM map point cloud")
        print("* N: Toggle SLAM map key frames")
        print("* G: Toggle 2D grid")
        print("* P: Toggle pose trail")
        print("* 0-5: Select point cloud color mode (ORIGINAL/X/Y/Z/DEPTH/NORMAL)")
        print("* H: Print this help window")
        print("------\n")
