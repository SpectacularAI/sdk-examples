"""
Mixed reality example using PyOpenGL. Requirements: spectacularai[full]

"""

import spectacularAI
from spectacularAI.cli.visualization.visualizer import Visualizer, VisualizerArgs, CameraMode
from mixed_reality import load_obj

from OpenGL.GL import * # all prefixed with gl so OK to import *

def parseArgs():
    import argparse
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("dataFolder", help="Folder containing the recorded session for replay", default="data")
    p.add_argument("--useRectification", help="This parameter must be set if the video inputs are not rectified", action="store_true")
    p.add_argument('--cameraInd', help="Which camera to use. Typically 0=left, 1=right, 2=auxiliary/RGB (OAK-D default)", type=int, default=2)
    p.add_argument("--mapLoadPath", help="SLAM map path", default=None)
    p.add_argument('--objLoadPath', help="Load scene as .obj", default=None)
    p.add_argument('--latitude', help="Scene coordinate system geographic origin (WGS84): latitude in degrees", default=None)
    p.add_argument('--longitude', help="Scene coordinate system geographic origin (WGS84): longitude in degrees", default=None)
    p.add_argument('--altitude', help="Scene coordinate system geographic origin (WGS84): altitude in meters", default=None)
    p.add_argument("--recordWindow", help="Window recording filename")
    return p.parse_args()

if __name__ == '__main__':
    args = parseArgs()

    configInternal = {}
    if args.useRectification:
        configInternal["useRectification"] = "true" # Undistort images for visualization (assumes undistorted pinhole model)

    if args.mapLoadPath:
        configInternal["mapLoadPath"] = args.mapLoadPath

    obj = None
    objPos = None # Position in WGS84 coordinates when GPS fusion is enabled
    if args.latitude and args.longitude and args.altitude:
        objPos = spectacularAI.WgsCoordinates()
        objPos.latitude = float(args.latitude)
        objPos.longitude = float(args.longitude)
        objPos.altitude = float(args.altitude)

    def renderObj():
        global obj
        if obj is None:
            obj = load_obj(args.objLoadPath)

        glColor3f(1, 0, 1)
        glLineWidth(2.0)
        glCallList(obj)

    visArgs = VisualizerArgs()
    visArgs.recordPath = args.recordWindow
    visArgs.cameraMode = CameraMode.AR
    visArgs.showPoseTrail = False
    visArgs.showKeyFrames = False
    visArgs.showGrid = False
    visArgs.customRenderCallback = renderObj
    visualizer = Visualizer(visArgs)

    def replayOnVioOutput(output, frameSet):
        if output.globalPose and objPos == None:
            # If we receive global pose i.e. recording contains GPS coordinates, then
            # place object at the first received device coordinates if not provide
            # through CLI arguments
            objPos = output.globalPose.coordinates

        for frame in frameSet:
            if not frame.image: continue
            if not frame.index == args.cameraInd: continue
            img = frame.image.toArray()
            width = img.shape[1]
            height = img.shape[0]
            colorFormat = frame.image.getColorFormat()

            if output.globalPose:
                # Gives camera pose relative to objPos that sits at [0,0,0] in ENU
                cameraPose = output.globalPose.getEnuCameraPose(args.cameraInd, objPos)
            else:
                cameraPose = frame.cameraPose

            visualizer.onVioOutput(cameraPose, img, width, height, colorFormat, output.status)

    replay = spectacularAI.Replay(args.dataFolder, configuration=configInternal)
    replay.setExtendedOutputCallback(replayOnVioOutput)
    replay.startReplay()
    visualizer.run()
    replay.close()
