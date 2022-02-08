"""
Mixed reality example using PyOpenGL. Requirements:

    pip install pygame PyOpenGL PyOpenGL_accelerate

"""
import depthai
import spectacularAI
import pygame
import time

from OpenGL.GL import * # all prefixed with gl so OK to import *

FPS = 24 # set to 30 for smoother frame rate

def parse_args():
    import argparse
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("--mapLoadPath", help="SLAM map path", default=None)
    return p.parse_args()

def make_pipelines():
    pipeline = depthai.Pipeline()
    config = spectacularAI.depthai.Configuration()
    args = parse_args()
    if args.mapLoadPath is not None:
        config.mapLoadPath = args.mapLoadPath
        config.useSlam = True
    vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, config)

    # NOTE: this simple method of reading RGB data from the device does not
    # scale to well to higher resolutions. Use YUV data with larger resolutions
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
    camRgb.initialControl.setManualFocus(130) # seems to be about 1m
    out_source = camRgb.preview

    xout_camera = pipeline.createXLinkOut()
    xout_camera.setStreamName("cam_out")
    out_source.link(xout_camera.input)

    return (pipeline, vio_pipeline)

def init_display(w, h):
    from pygame.locals import DOUBLEBUF, OPENGL
    pygame.init()
    pygame.display.set_mode((w, h), DOUBLEBUF | OPENGL)

CUBE_VERTICES = (
    (1, -1, -1), (1, 1, -1), (-1, 1, -1), (-1, -1, -1),
    (1, -1, 1), (1, 1, 1), (-1, -1, 1), (-1, 1, 1)
)

CUBE_EDGES = (
    (0,1), (0,3), (0,4), (2,1), (2,3), (2,7),
    (6,3), (6,4), (6,7), (5,1), (5,4), (5,7)
)

def draw_cube():
    glBegin(GL_LINES)
    for edge in CUBE_EDGES:
        for vertex in edge:
            glVertex3fv(CUBE_VERTICES[vertex])
    glEnd()

def draw(cam, img):
    # copy image as AR background
    glDrawPixels(img.getWidth(), img.getHeight(), GL_RGB, GL_UNSIGNED_BYTE, img.getRaw().data)

    # setup OpenGL camera based on VIO output
    glLoadIdentity()
    near, far = 0.01, 30.0 # clip
    glMultMatrixd(cam.camera.getProjectionMatrixOpenGL(near, far).transpose())
    glMultMatrixd(cam.getWorldToCameraMatrix().transpose())

    # cube world position
    glTranslatef(0.5, 0, 0);
    glScalef(*([0.1] * 3))

    glClear(GL_DEPTH_BUFFER_BIT)
    glColor3f(1, 0, 1);
    glLineWidth(2.0)
    draw_cube()

pipeline, vio_pipeline = make_pipelines()

def main_loop(device, vio_session):
    display_initialized = False
    img_queue = device.getOutputQueue(name="cam_out", maxSize=4, blocking=False)

    # buffer for frames: show together with the corresponding VIO output
    frames = {}
    frame_number = 1

    while True:
        if img_queue.has():
            img = img_queue.get()
            img_time = img.getTimestamp().total_seconds()
            vio_session.addTrigger(img_time, frame_number)
            frames[frame_number] = img
            frame_number += 1

        elif vio_session.hasOutput():
            out = vio_session.getOutput()

            if out.tag > 0:
                img = frames.get(out.tag)

                if not display_initialized:
                    display_initialized = True
                    clock = pygame.time.Clock()
                    init_display(img.getWidth(), img.getHeight())

                cam = vio_session.getRgbCameraPose(out)

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        return

                draw(cam, img)

                pygame.display.flip()
                # uncomment for smooth frame rate at higher latency
                # clock.tick(FPS)

                # discard old tags
                frames = { tag: v for tag, v in frames.items() if tag > out.tag }
        else:
            pygame.time.wait(1)

with depthai.Device(pipeline) as device, \
    vio_pipeline.startSession(device) as vio_session:
    main_loop(device, vio_session)
