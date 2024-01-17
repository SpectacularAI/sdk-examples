"""
Mixed reality example using PyOpenGL. Requirements:

    pip install pygame PyOpenGL PyOpenGL_accelerate opencv-python

For AprilTag mode, see: https://github.com/SpectacularAI/docs/blob/main/pdf/april_tag_instructions.pdf
"""

import depthai
import spectacularAI
import pygame
import time

from OpenGL.GL import * # all prefixed with gl so OK to import *

FPS = 24 # set to 30 for smoother frame rate
CAM_RESOLUTION = depthai.ColorCameraProperties.SensorResolution.THE_1080_P
CAM_SOCKET = depthai.CameraBoardSocket.CAM_A
ISP_SCALE = (1,2)

def parse_args():
    import argparse
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("--mapLoadPath", help="SLAM map path", default=None)
    p.add_argument('--objLoadPath', help="Load scene as .obj", default=None)
    p.add_argument('--aprilTagPath', help="Path to .json file with AprilTag ids, sizes and poses", default=None)
    return p.parse_args()

# https://github.com/luxonis/depthai-python/blob/855e0208361d60ad250bc83ae64ce7d44e23140d/examples/ColorCamera/rgb_undistort.py#L9
def getMesh(calibData, ispSize):
    import numpy as np
    import cv2

    M1 = np.array(calibData.getCameraIntrinsics(CAM_SOCKET, ispSize[0], ispSize[1]))
    d1 = np.array(calibData.getDistortionCoefficients(CAM_SOCKET))
    R1 = np.identity(3)
    mapX, mapY = cv2.initUndistortRectifyMap(M1, d1, R1, M1, ispSize, cv2.CV_32FC1)

    meshCellSize = 16
    mesh0 = []
    # Creates subsampled mesh which will be loaded on to device to undistort the image
    for y in range(mapX.shape[0] + 1): # iterating over height of the image
        if y % meshCellSize == 0:
            rowLeft = []
            for x in range(mapX.shape[1]): # iterating over width of the image
                if x % meshCellSize == 0:
                    if y == mapX.shape[0] and x == mapX.shape[1]:
                        rowLeft.append(mapX[y - 1, x - 1])
                        rowLeft.append(mapY[y - 1, x - 1])
                    elif y == mapX.shape[0]:
                        rowLeft.append(mapX[y - 1, x])
                        rowLeft.append(mapY[y - 1, x])
                    elif x == mapX.shape[1]:
                        rowLeft.append(mapX[y, x - 1])
                        rowLeft.append(mapY[y, x - 1])
                    else:
                        rowLeft.append(mapX[y, x])
                        rowLeft.append(mapY[y, x])
            if (mapX.shape[1] % meshCellSize) % 2 != 0:
                rowLeft.append(0)
                rowLeft.append(0)

            mesh0.append(rowLeft)

    mesh0 = np.array(mesh0)
    meshWidth = mesh0.shape[1] // 2
    meshHeight = mesh0.shape[0]
    mesh0.resize(meshWidth * meshHeight, 2)

    mesh = list(map(tuple, mesh0))

    return mesh, meshWidth, meshHeight

def make_pipelines(config, onMappingOutput=None):
    pipeline = depthai.Pipeline()
    vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, config, onMappingOutput)

    # read rgb calibration
    with depthai.Device() as device:
        calibData = device.readCalibration()

    camRgb = pipeline.create(depthai.node.ColorCamera)
    camRgb.setBoardSocket(CAM_SOCKET)
    camRgb.setResolution(CAM_RESOLUTION)
    camRgb.setIspScale(ISP_SCALE)
    camRgb.setImageOrientation(depthai.CameraImageOrientation.VERTICAL_FLIP) # for OpenGL
    camRgb.setFps(FPS)
    camRgb.initialControl.setAutoFocusMode(depthai.RawCameraControl.AutoFocusMode.OFF)
    camRgb.initialControl.setManualFocus(130) # seems to be about 1m

    # undistort
    manipUndistort = pipeline.create(depthai.node.ImageManip)
    mesh, meshWidth, meshHeight = getMesh(calibData, camRgb.getIspSize())
    manipUndistort.setWarpMesh(mesh, meshWidth, meshHeight)
    manipUndistort.setMaxOutputFrameSize(camRgb.getIspWidth() * camRgb.getIspHeight() * 3 // 2)
    camRgb.isp.link(manipUndistort.inputImage)

    # yuv->rgb
    manipRgb = pipeline.create(depthai.node.ImageManip)
    manipRgb.setMaxOutputFrameSize(1555200)
    manipRgb.initialConfig.setFrameType(depthai.ImgFrame.Type.RGB888i)
    manipUndistort.out.link(manipRgb.inputImage)

    cam_xout = pipeline.create(depthai.node.XLinkOut)
    cam_xout.setStreamName("cam_out")
    manipRgb.out.link(cam_xout.input)

    return (pipeline, vio_pipeline)

def init_display(w, h):
    from pygame.locals import DOUBLEBUF, OPENGL
    pygame.init()
    pygame.display.set_mode((w, h), DOUBLEBUF | OPENGL)

def draw_cube(origin):
    CUBE_VERTICES = (
        (1, -1, -1), (1, 1, -1), (-1, 1, -1), (-1, -1, -1),
        (1, -1, 1), (1, 1, 1), (-1, -1, 1), (-1, 1, 1)
    )

    CUBE_EDGES = (
        (0,1), (0,3), (0,4), (2,1), (2,3), (2,7),
        (6,3), (6,4), (6,7), (5,1), (5,4), (5,7)
    )
    glPushMatrix()
    # cube world position
    glTranslatef(origin[0], origin[1], origin[2])
    glScalef(*([0.1] * 3))

    glBegin(GL_LINES)
    for edge in CUBE_EDGES:
        for vertex in edge:
            glVertex3fv(CUBE_VERTICES[vertex])
    glEnd()
    glPopMatrix()

def load_and_draw_obj_as_wireframe(in_stream):
    vertices = []
    glBegin(GL_LINES)
    for line in in_stream:
        if line.startswith('#'): continue
        cmd, _, rest = line.partition(' ')
        data = rest.split()
        if cmd == 'v':
            vertices.append([float(c) for c in data])
        elif cmd == 'f':
            indices = [int(c.split('/')[0]) for c in data]
            for i in range(len(indices)):
                glVertex3fv(vertices[indices[i] - 1])
                glVertex3fv(vertices[indices[(i + 1) % len(indices)] - 1])
        # skip everything else
    glEnd()

def load_obj(objLoadPath, origin=(0.5, 0, 0)):
    gl_list = glGenLists(1)
    glNewList(gl_list, GL_COMPILE)
    if objLoadPath is None:
        draw_cube(origin)
    else:
        with open(objLoadPath, 'r') as f:
            load_and_draw_obj_as_wireframe(f)
    glEndList()
    return gl_list

def draw(cam, width, height, data, obj, is_tracking):
    # copy image as AR background
    glDrawPixels(width, height, GL_RGB, GL_UNSIGNED_BYTE, data)
    if not is_tracking: return

    # setup OpenGL camera based on VIO output
    glLoadIdentity()
    near, far = 0.01, 100.0 # clip
    glMultMatrixd(cam.camera.getProjectionMatrixOpenGL(near, far).transpose())
    glMultMatrixd(cam.getWorldToCameraMatrix().transpose())

    glClear(GL_DEPTH_BUFFER_BIT)
    glColor3f(1, 0, 1)
    glLineWidth(2.0)
    glCallList(obj)

def main_loop(args, device, vio_session):
    display_initialized = False
    img_queue = device.getOutputQueue(name="cam_out", maxSize=4, blocking=False)

    # buffer for frames: show together with the corresponding VIO output
    frames = {}
    frame_number = 1
    obj = None

    while True:
        if img_queue.has():
            img = img_queue.get()
            img_time = img.getTimestampDevice().total_seconds()
            frames[frame_number] = img
            vio_session.addTrigger(img_time, frame_number)
            frame_number += 1

        elif vio_session.hasOutput():
            out = vio_session.getOutput()

            if out.tag > 0:
                img = frames.get(out.tag)

                if not display_initialized:
                    display_initialized = True
                    clock = pygame.time.Clock()
                    init_display(img.getWidth(), img.getHeight())
                    origin = (0, 0, 0) if args.aprilTagPath is not None else (0.5, 0, 0)
                    obj = load_obj(args.objLoadPath, origin)

                cam = vio_session.getRgbCameraPose(out)

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        return

                is_tracking = out.status == spectacularAI.TrackingStatus.TRACKING
                draw(cam, img.getWidth(), img.getHeight(), img.getRaw().data, obj, is_tracking)

                pygame.display.flip()
                # uncomment for smooth frame rate at higher latency
                # clock.tick(FPS)

                # discard old tags
                frames = { tag: v for tag, v in frames.items() if tag > out.tag }
        else:
            pygame.time.wait(1)

if __name__ == '__main__':
    args = parse_args()

    config = spectacularAI.depthai.Configuration()
    if args.mapLoadPath is not None:
        config.mapLoadPath = args.mapLoadPath
        config.useSlam = True
    elif args.aprilTagPath is not None:
        config.aprilTagPath = args.aprilTagPath

    pipeline, vio_pipeline = make_pipelines(config)
    with depthai.Device(pipeline) as device, \
        vio_pipeline.startSession(device) as vio_session:
        main_loop(args, device, vio_session)
