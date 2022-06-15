"""
Mixed reality example using PyOpenGL. Requirements:

    pip install pygame PyOpenGL PyOpenGL_accelerate

"""
import depthai
import spectacularAI
import pygame
import time
import numpy as np
from mixed_reality import init_display, draw_cube, draw, load_obj

from OpenGL.GL import * # all prefixed with gl so OK to import *

def parse_args():
    import argparse
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("dataFolder", help="Folder containing the recorded session for replay", default="data")
    p.add_argument("--mapLoadPath", help="SLAM map path", default=None)
    p.add_argument('--objLoadPath', help="Load scene as .obj", default=None)
    p.add_argument('--latitude', help="Objects latitude coordinate", default=None)
    p.add_argument('--longitude', help="Objects longitude coordinate", default=None)
    p.add_argument('--altitude', help="Objects altitude in meters", default=None)
    return p.parse_args()
args = parse_args()

display_initialized = False
obj = None
objPos = None
if args.latitude and args.longitude and args.altitude:
    objPos = spectacularAI.WgsCoordinates()
    objPos.latitude = float(args.latitude)
    objPos.longitude = float(args.longitude)
    objPos.altitude = float(args.altitude)

def onOutput(output, frameSet):
    global display_initialized
    global obj
    global objPos
    global args
    if not output.globalPose:
        print("[{:.2f}] No GNSS pose yet, waiting for long enough trail to align Vio and GNSS tracks".format(output.pose.time))
        return

    if objPos == None:
        # Place object at the first received device coordinates if not provide through CLI arguments
        objPos = output.globalPose.coordinates

    for frame in frameSet:
        if frame.image.getColorFormat() == spectacularAI.ColorFormat.RGB:
            img = frame.image.toArray()
            # Flip the image upside down for OpenGL
            img = np.ascontiguousarray(np.flipud(img))
            width = img.shape[1]
            height = img.shape[0]

            if not display_initialized:
                display_initialized = True
                init_display(width, height)
                obj = load_obj(args.objLoadPath)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return

            cameraIndex = 0
            # Gives camera pose relative to objPos that sits at [0,0,0] in ENU
            cameraPose = output.globalPose.getEnuCameraPose(cameraIndex, objPos)

            is_tracking = output.status == spectacularAI.TrackingStatus.TRACKING
            draw(cameraPose, width, height, img.data, obj, is_tracking)
            pygame.display.flip()
            break # Skip other frames

replay = spectacularAI.Replay(args.dataFolder)
replay.setExtendedOutputCallback(onOutput)
replay.startReplay()
