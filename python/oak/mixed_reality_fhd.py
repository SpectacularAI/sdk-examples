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
WINDOW_SCALE = 1.0

def make_pipelines():
    pipeline = depthai.Pipeline()
    vio_pipeline = spectacularAI.depthai.Pipeline(pipeline)

    camRgb = pipeline.createColorCamera()

    camRgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setFps(FPS)

    camRgb.initialControl.setAutoFocusMode(depthai.RawCameraControl.AutoFocusMode.OFF)
    camRgb.initialControl.setManualFocus(130)
    out_source = camRgb.isp

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
            win_w, win_h = [round(x * WINDOW_SCALE) for x in (w, h)]
            pygame.display.set_mode((win_w, win_h), DOUBLEBUF | OPENGL)

            VERTEX_SHADER_FLIP_TEX_Y = """
            #version 120
            void main() {
                gl_Position = vec4(gl_Vertex.xy * 2.0 - vec2(1.0), 0.0, 1.0);
                gl_TexCoord[0] = vec4(gl_Vertex.x, 1.0 - gl_Vertex.y, 0.0, 1.0);
            }
            """

            FRAGMENT_SHADER_YUV_TO_RGB = """
            #version 120
            uniform sampler2D tex_y;
            uniform sampler2D tex_u;
            uniform sampler2D tex_v;
            void main()
            {
                vec2 texCoord = gl_TexCoord[0].st;
                float y = texture2D(tex_y, texCoord).r;
                float u = texture2D(tex_u, texCoord).r;
                float v = texture2D(tex_v, texCoord).r;
                const float y_offs = 16.0 / 255.0;
                gl_FragColor = vec4(
                    1.164 * (y - y_offs) + 1.596 * (v - 0.5),
                    1.164 * (y - y_offs) - 0.813 * (v - 0.5) - 0.391 * (u - 0.5),
                    1.164 * (y - y_offs) + 2.018 * (u - 0.5),
                    1.0);
            }
            """

            self.background_shader = create_gl_program(VERTEX_SHADER_FLIP_TEX_Y, FRAGMENT_SHADER_YUV_TO_RGB)
            self.background_vbo = vbo.VBO(np.array(Renderer.SCREEN_QUAD_TRIANGLE_FAN, 'f'))

            # YUV
            self.background_textures = [glGenTextures(1) for i in range(3)]
            self.background_texture_uniforms = []

            shaders.glUseProgram(self.background_shader)
            for i in range(len(self.background_textures)):
                glActiveTexture(GL_TEXTURE0)
                self.background_texture_uniforms.append(
                    glGetUniformLocation(self.background_shader, "tex_%s" % ("yuv"[i])))
                glBindTexture(GL_TEXTURE_2D, self.background_textures[i])
                downscale = 1
                if i > 0: downscale = 2
                tex_w, tex_h = w // downscale, h // downscale

                glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, tex_w, tex_h, 0, GL_RGB, GL_UNSIGNED_BYTE, None)
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

            glBindTexture(GL_TEXTURE_2D, 0)
            shaders.glUseProgram(0)

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
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)
        shaders.glUseProgram(self.background_shader)

        raw_yuv = img.getRaw().data
        for i in range(len(self.background_textures)):
            w, h = img.getWidth(), img.getHeight()
            offset = 0
            if i > 0:
                downscale = 2
                tex_w, tex_h = w // downscale, h // downscale
                offset = w * h + (tex_w * tex_h) * (i - 1)
            else:
                tex_w, tex_h = w, h

            data = raw_yuv[offset:(offset+tex_w*tex_h)]

            glActiveTexture(GL_TEXTURE0 + i)
            glBindTexture(GL_TEXTURE_2D, self.background_textures[i])
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, tex_w, tex_h, GL_LUMINANCE, GL_UNSIGNED_BYTE, data)
            glUniform1i(self.background_texture_uniforms[i], i)

        self.background_vbo.bind()
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointerf(self.background_vbo)

        glDrawArrays(GL_TRIANGLE_FAN, 0, len(Renderer.SCREEN_QUAD_TRIANGLE_FAN))

        glDisableClientState(GL_VERTEX_ARRAY)
        self.background_vbo.unbind()

        for i in range(len(self.background_textures)):
            glActiveTexture(GL_TEXTURE0 + i)
            glBindTexture(GL_TEXTURE_2D, 0)

        glActiveTexture(GL_TEXTURE0)
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
