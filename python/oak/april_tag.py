"""
April Tag example. Visualizes April Tag detections in SLAM keyframes, and April Tag poses using augmented reality.

For April Tag instructions, see: https://github.com/SpectacularAI/docs/blob/main/pdf/april_tag_instructions.pdf

Requirements: pip install spectacularAI[full]
"""

import depthai
import spectacularAI
import cv2
import threading
import time
import numpy as np
from mixed_reality import make_pipelines

from spectacularAI.cli.visualization.visualizer import Visualizer, VisualizerArgs, CameraMode

from OpenGL.GL import * # all prefixed with gl so OK to import *

def parseArgs():
    import argparse
    p = argparse.ArgumentParser(__doc__)
    p.add_argument('aprilTagPath', help="Path to .json file with AprilTag ids, sizes and poses")
    p.add_argument("--dataFolder", help="Instead of running live mapping session, replay session from this folder")
    p.add_argument("--useRectification", help="This parameter must be set if the videos inputs are not rectified", action="store_true")
    p.add_argument('--cameraInd', help="Which camera to use with Replay mode. Typically 0=left, 1=right, 2=auxiliary/RGB (OAK-D default)", type=int, default=2)
    return p.parse_args()

def drawAprilTag(camera, img, tag):
    def projectTagPointToPixel(camera, pTag, tagToCamera):
        pCamera = tagToCamera[:3, :3] @ pTag  + tagToCamera[:3, 3]
        pixel = camera.rayToPixel(spectacularAI.Vector3d(pCamera[0], pCamera[1], pCamera[2]))
        if pixel is None: return None
        return (int(pixel.x), int(pixel.y))

    def drawEdges(img, p1, p2, p3, p4, color, thickness):
        cv2.line(img, p1, p2, color, thickness)
        cv2.line(img, p2, p3, color, thickness)
        cv2.line(img, p3, p4, color, thickness)
        cv2.line(img, p4, p1, color, thickness)

    # Draw detected April Tag with blue
    p1 = (int(tag.corners[0].x), int(tag.corners[0].y))
    p2 = (int(tag.corners[1].x), int(tag.corners[1].y))
    p3 = (int(tag.corners[2].x), int(tag.corners[2].y))
    p4 = (int(tag.corners[3].x), int(tag.corners[3].y))
    drawEdges(img, p1, p2, p3, p4, (0, 0, 255), 2)

    # (Currently) pose and size is known only for April Tags defined in tags.json
    if not tag.hasPose: return

    # Project April Tag corners points to image using estimated pose
    r = tag.size / 2
    tagToCamera = np.linalg.inv(tag.pose.asMatrix())
    p1 = projectTagPointToPixel(camera, (-r, r, 0), tagToCamera)
    p2 = projectTagPointToPixel(camera, (r, r, 0), tagToCamera)
    p3 = projectTagPointToPixel(camera, (r, -r, 0), tagToCamera)
    p4 = projectTagPointToPixel(camera, (-r, -r, 0), tagToCamera)
    drawEdges(img, p1, p2, p3, p4, (255, 0, 0), 1)

    # Project AR axis using estimated pose
    center = projectTagPointToPixel(camera, (0, 0, 0), tagToCamera)
    right = projectTagPointToPixel(camera, (r, 0, 0), tagToCamera)
    down = projectTagPointToPixel(camera, (0, r, 0), tagToCamera)
    into = projectTagPointToPixel(camera, (0, 0, r), tagToCamera)

    cv2.line(img, center, right, (255, 0, 0), 2)
    cv2.line(img, center, down, (0, 255, 0), 2)
    cv2.line(img, center, into, (0, 0, 255), 2)

# Draws April Tags detected in the latest key frame
def onMappingOutput(output):
    if not output.map.keyFrames: return # empty map

    # Find latest keyframe
    keyFrameId = max(output.map.keyFrames.keys())
    keyFrame = output.map.keyFrames[keyFrameId]
    frameSet = keyFrame.frameSet

    def drawAprilTagsInFrame(frame):
        if frame is None or frame.image is None: return None
        img = cv2.cvtColor(frame.image.toArray(), cv2.COLOR_RGB2BGR)
        for marker in frame.visualMarkers:
            drawAprilTag(frame.cameraPose.camera, img, marker)
        return img

    primary = drawAprilTagsInFrame(frameSet.primaryFrame)
    secondary = drawAprilTagsInFrame(frameSet.secondaryFrame)

    if primary is None and secondary is None: return
    elif primary is None: img = secondary
    elif secondary is None: img = primary
    else: img = cv2.hconcat([primary, secondary])

    cv2.imshow("April Tags", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    cv2.setWindowTitle("April Tags", "April Tags detected in key frame #{}".format(keyFrameId))
    cv2.waitKey(1)

def drawAxis(localToWorld):
    AXIS_LINES = (
        (0, 0, 0), (1, 0, 0), # X axis
        (0, 0, 0), (0, 1, 0), # Y axis
        (0, 0, 0), (0, 0, 1)  # Z axis
    )

    glPushMatrix()
    glMultMatrixf(localToWorld.transpose())
    glScalef(*([0.1] * 3))

    glBegin(GL_LINES)
    glColor3f(1, 0, 0)
    glVertex3fv(AXIS_LINES[0])
    glVertex3fv(AXIS_LINES[1])

    glColor3f(0, 1, 0)
    glVertex3fv(AXIS_LINES[2])
    glVertex3fv(AXIS_LINES[3])

    glColor3f(0, 0, 1)
    glVertex3fv(AXIS_LINES[4])
    glVertex3fv(AXIS_LINES[5])
    glEnd()

    glPopMatrix()

def loadAprilTagObjs(aprilTagPath):
    def readAprilTagPoses(aprilTagPath):
        import json
        poses = []
        with open(aprilTagPath) as f:
            tags = json.load(f)
            for tag in tags:
                tagToWorld = np.array(tag['tagToWorld'])
                poses.append(tagToWorld)
        return poses

    poses = readAprilTagPoses(aprilTagPath)
    glList = glGenLists(1)
    glNewList(glList, GL_COMPILE)
    for pose in poses:
        drawAxis(pose)
    glEndList()
    return glList

def replayOnVioOutput(output, frameSet):
    for frame in frameSet:
        if not frame.image: continue
        if not frame.index == args.cameraInd: continue
        img = frame.image.toArray()
        width = img.shape[1]
        height = img.shape[0]
        colorFormat = frame.image.getColorFormat()
        cameraPose = frame.cameraPose
        visualizer.onVioOutput(cameraPose, img, width, height, colorFormat, output.status)

if __name__ == '__main__':
    args = parseArgs()

    obj = None
    def renderObj():
        global obj
        if obj is None:
            obj = loadAprilTagObjs(args.aprilTagPath)

        glColor3f(1, 0, 1)
        glLineWidth(2.0)
        glCallList(obj)

    visArgs = VisualizerArgs()
    visArgs.cameraMode = CameraMode.AR
    visArgs.showPoseTrail = False
    visArgs.showKeyFrames = False
    visArgs.showGrid = False
    visArgs.customRenderCallback = renderObj
    visualizer = Visualizer(visArgs)

    if args.dataFolder:
        configInternal = {
            "aprilTagPath": args.aprilTagPath,
            "extendParameterSets" : ["april-tags"]
        }
        if args.useRectification:
            configInternal["useRectification"] = "true" # Undistort images for visualization (assumes undistorted pinhole model)

        replay = spectacularAI.Replay(args.dataFolder, onMappingOutput, configuration=configInternal)
        replay.setExtendedOutputCallback(replayOnVioOutput)
        replay.startReplay()
        visualizer.run()
        replay.close()
    else:
        def captureLoop():
            config = spectacularAI.depthai.Configuration()
            config.aprilTagPath = args.aprilTagPath

            pipeline, vioPipeline = make_pipelines(config, onMappingOutput)
            with depthai.Device(pipeline) as device, \
                vioPipeline.startSession(device) as vioSession:

                # buffer for frames: show together with the corresponding VIO output
                frames = {}
                frameNumber = 1
                imgQueue = device.getOutputQueue(name="cam_out", maxSize=4, blocking=False)

                while not visualizer.shouldQuit:
                    if imgQueue.has():
                        img = imgQueue.get()
                        imgTime = img.getTimestampDevice().total_seconds()
                        frames[frameNumber] = img
                        vioSession.addTrigger(imgTime, frameNumber)
                        frameNumber += 1

                    elif vioSession.hasOutput():
                        output = vioSession.getOutput()

                        if output.tag > 0:
                            import numpy as np
                            img = frames.get(output.tag)
                            data = np.flipud(img.getRaw().data)
                            cameraPose = vioSession.getRgbCameraPose(output)
                            visualizer.onVioOutput(cameraPose, data, img.getWidth(), img.getHeight(), spectacularAI.ColorFormat.RGB, output.status)
                            # discard old tags
                            frames = { tag: v for tag, v in frames.items() if tag > output.tag }
                    time.sleep(0.01)

        thread = threading.Thread(target=captureLoop)
        thread.start()
        visualizer.run()
        thread.join()
