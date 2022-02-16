"""
Mixed reality example using PyOpenGL. Requirements:

    pip install pygame PyOpenGL PyOpenGL_accelerate

"""
import depthai
import spectacularAI
import pygame
import time
import numpy as np
# import init_display from mixed_reality
from mixed_reality import init_display, draw_cube, draw, load_obj

from OpenGL.GL import * # all prefixed with gl so OK to import *

def parse_args():
    import argparse
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("dataFolder", help="Folder containing the recorded session for replay", default="data")
    p.add_argument("--mapLoadPath", help="SLAM map path", default=None)
    p.add_argument('--objLoadPath', help="Load scene as .obj", default=None)
    return p.parse_args()
args = parse_args()

display_initialized = False
obj = None

def onOutput(output, frameSet):
    global display_initialized
    global obj
    global args
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

            draw(frame.cameraPose, width, height, img.data, obj)
            pygame.display.flip()
            break # Skip other frames

replay = spectacularAI.Replay(args.dataFolder)
replay.setExtendedOutputCallback(onOutput)
replay.startReplay()
