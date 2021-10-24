"""
More complex mixed reality example using PyOpenGL.

    pip install pygame PyOpenGL PyOpenGL_accelerate

"""
import depthai
import math
import spectacularAI
import pygame
import threading
import time

from OpenGL.GL import * # all prefixed with gl so OK to import *

FPS = 30

def make_pipelines():
    pipeline = depthai.Pipeline()
    vio_pipeline = spectacularAI.depthai.Pipeline(pipeline)

    RGB_OUTPUT_WIDTH = 1024
    REF_ASPECT = 1920 / 1080.0
    w = RGB_OUTPUT_WIDTH
    h = int(round(w / REF_ASPECT))

    camRgb = pipeline.createColorCamera()
    camRgb.setPreviewSize(w, h)
    camRgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setColorOrder(depthai.ColorCameraProperties.ColorOrder.RGB)
    camRgb.setImageOrientation(depthai.CameraImageOrientation.VERTICAL_FLIP) # for OpenGL
    camRgb.setFps(FPS)
    camRgb.initialControl.setAutoFocusMode(depthai.RawCameraControl.AutoFocusMode.OFF)
    camRgb.initialControl.setManualFocus(130)
    out_source = camRgb.preview

    xout_camera = pipeline.createXLinkOut()
    xout_camera.setStreamName("cam_out")
    out_source.link(xout_camera.input)

    return (pipeline, vio_pipeline)

class CubeModel:
    def __init__(self):
        self.vertices = (
            (1, -1, -1), (1, 1, -1), (-1, 1, -1), (-1, -1, -1),
            (1, -1, 1), (1, 1, 1), (-1, -1, 1), (-1, 1, 1)
        )

        self.edges = (
            (0,1), (0,3), (0,4), (2,1), (2,3), (2,7),
            (6,3), (6,4), (6,7), (5,1), (5,4), (5,7)
        )

    def draw(self):
        # cube world position
        glTranslatef(0.5, 0, 0);
        glScalef(*([0.1] * 3))

        glColor3f(1, 0, 1);
        glLineWidth(2.0)

        glBegin(GL_LINES)
        for edge in self.edges:
            for vertex in edge:
                glVertex3fv(self.vertices[vertex])
        glEnd()

class Renderer:
    def __init__(self, w, h):
        self.image_queue = []
        self.vio_queue = []
        self.should_quit = False

        def render_loop():
            from pygame.locals import DOUBLEBUF, OPENGL
            pygame.init()
            pygame.display.set_mode((w, h), DOUBLEBUF | OPENGL)

            self.model = CubeModel()
            clock = pygame.time.Clock()
            prev_frame_time = 0

            background_frame_number = 0
            background_used = True

            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        self.should_quit = True
                        return

                # print(len(self.image_queue), len(self.vio_queue))
                if background_used and len(self.image_queue) > 0:
                    img, background_frame_number = self.image_queue.pop(0)
                    self._draw_background(img)
                    background_used = False

                if not background_used and len(self.vio_queue) > 0:
                    cam, frame_number = self.vio_queue.pop(0)
                    assert(frame_number >= background_frame_number)
                    if frame_number == background_frame_number:
                        self._draw_model(cam)
                        pygame.display.flip()

                        cur_time = time.monotonic()
                        # print("dt %gms" % ((cur_time - prev_frame_time)*1000))
                        prev_frame_time = cur_time
                        clock.tick(FPS)
                    else:
                        print('warn: skipped input frame')
                    background_used = True

                pygame.time.wait(1)

        self.thread = threading.Thread(target = render_loop)
        self.thread.start()

    def push_image(self, img, tag):
        self.image_queue.append((img, tag))

    def push_vio(self, cam, tag):
        if self.should_quit:
            self.thread.join()
            return False
        self.vio_queue.append((cam, tag))
        return True

    def _draw_background(self, img):
        # copy image as AR background
        glDrawPixels(img.getWidth(), img.getHeight(), GL_RGB, GL_UNSIGNED_BYTE, img.getRaw().data)

    def _draw_model(self, cam):
        # setup OpenGL camera based on VIO output
        glLoadIdentity()
        near, far = 0.01, 30.0 # clip
        glMultMatrixd(cam.camera.getProjectionMatrixOpenGL(near, far).transpose())
        glMultMatrixd(cam.getWorldToCameraMatrix().transpose())
        glClear(GL_DEPTH_BUFFER_BIT)
        self.model.draw()

pipeline, vio_pipeline = make_pipelines()

with depthai.Device(pipeline) as device, \
    vio_pipeline.startSession(device) as vio_session:

    renderer = None
    img_queue = device.getOutputQueue(name="cam_out", maxSize=4, blocking=False)
    frame_number = 1

    while True:
        if img_queue.has():
            img = img_queue.get()
            img_time = img.getTimestamp().total_seconds()
            vio_session.addTrigger(img_time, frame_number)

            if renderer is None:
                renderer = Renderer(img.getWidth(), img.getHeight())

            renderer.push_image(img, frame_number)
            frame_number += 1

        elif vio_session.hasOutput():
            out = vio_session.getOutput()
            if out.tag > 0:
                assert(renderer is not None)
                cam = vio_session.getRgbCameraPose(out)
                if not renderer.push_vio(cam, out.tag): break
        else:
            time.sleep(0.005)
