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
import numpy as np

from OpenGL.GL import * # all prefixed with gl so OK to import *
from OpenGL.arrays import vbo
from OpenGL.GL import shaders

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

def create_gl_program(vertex_source, fragment_source):
    vertex_shader = shaders.compileShader(vertex_source, GL_VERTEX_SHADER)
    fragment_shader = shaders.compileShader(fragment_source, GL_FRAGMENT_SHADER)
    return shaders.compileProgram(vertex_shader, fragment_shader)

class Renderer:
    SCREEN_QUAD_TRIANGLE_FAN = [[x, y, 0] for x, y in [
      [0, 0],
      [0, 1],
      [1, 1],
      [1, 0]
    ]]

    def __init__(self, w, h):
        self.image_queue = []
        self.vio_queue = []
        self.should_quit = False

        def render_loop():
            from pygame.locals import DOUBLEBUF, OPENGL
            pygame.init()
            pygame.display.set_mode((w, h), DOUBLEBUF | OPENGL)

            VERTEX_SHADER_FLIP_TEX_Y = """
            #version 120
            void main() {
                gl_Position = vec4(gl_Vertex.xy * 2.0 - vec2(1.0), 0.0, 1.0);
                gl_TexCoord[0] = vec4(gl_Vertex.x, 1.0 - gl_Vertex.y, 0.0, 1.0);
            }
            """

            TRIVIAL_FRAGMENT_SHADER = """
            #version 120
            uniform sampler2D tex;
            void main()
            {
                vec4 color = texture2D(tex, gl_TexCoord[0].st);
                // vec4 color = vec4(gl_TexCoord[0].st, 0.0, 1.0);
                gl_FragColor = color;
            }
            """

            self.background_shader = create_gl_program(VERTEX_SHADER_FLIP_TEX_Y, TRIVIAL_FRAGMENT_SHADER)
            self.background_vbo = vbo.VBO(np.array(Renderer.SCREEN_QUAD_TRIANGLE_FAN, 'f'))

            self.background_texture = glGenTextures(1)

            shaders.glUseProgram(self.background_shader)
            glUniform1i(glGetUniformLocation(self.background_shader, "tex"), 0)
            shaders.glUseProgram(0)

            glActiveTexture(GL_TEXTURE0)
            glBindTexture(GL_TEXTURE_2D, self.background_texture)

            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0,  GL_RGB, GL_UNSIGNED_BYTE, None)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

            glBindTexture(GL_TEXTURE_2D, 0)


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
        glActiveTexture(GL_TEXTURE0)
        glBindTexture(GL_TEXTURE_2D, self.background_texture)
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, img.getWidth(), img.getHeight(), GL_RGB, GL_UNSIGNED_BYTE, img.getRaw().data)

        # copy image as AR background
        #glDrawPixels(img.getWidth(), img.getHeight(), GL_RGB, GL_UNSIGNED_BYTE, img.getRaw().data)

        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)
        shaders.glUseProgram(self.background_shader)

        self.background_vbo.bind()
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointerf(self.background_vbo)

        glDrawArrays(GL_TRIANGLE_FAN, 0, len(Renderer.SCREEN_QUAD_TRIANGLE_FAN))

        glDisableClientState(GL_VERTEX_ARRAY)
        self.background_vbo.unbind()

        glBindTexture(GL_TEXTURE_2D, 0)
        shaders.glUseProgram(0)

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
