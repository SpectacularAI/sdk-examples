import argparse
import spectacularAI
import cv2
import time
import threading
import os
from enum import Enum

class State(Enum):
    FindAprilTagTime = 1
    FindRelocalizeTime = 2
    FindAprilTagPose = 3
    FindRelocalizePose = 4

pairedFrameTime = None
aprilTagFrameTime = None
aprilTagFramePose = None
relocalizedFrameTime = None
relocalizedFramePose = None
args = None
aprilTagReplay1 = None
aprilTagReplay2 = None
relocalizeReplay1 = None
relocalizeReplay2 = None
state = None

def onAprilTagExtendedOutput(output, frameSet):
    global args, aprilTagFrameTime, aprilTagFramePose, aprilTagReplay1, aprilTagReplay2, pairedFrameTime, state
    # print(output.status)
    if not output.status == spectacularAI.TrackingStatus.TRACKING: return # not tracking yet

    for frame in frameSet:
        if not pairedFrameTime:
            if not aprilTagFrameTime:
                aprilTagFrameTime = output.pose.time
                print("Tracking started with April Tag at: {}".format(aprilTagFrameTime))
            # print("Close April Tag replay")
            aprilTagReplay1.close()
            break
        elif state == State.FindAprilTagPose and output.pose.time >= pairedFrameTime:
            if aprilTagFramePose is None:
                aprilTagFramePose = frame.cameraPose.pose.asMatrix()
                print("April Tag pose at: {}\n{}".format(output.pose.time, aprilTagFramePose))
            # print("Close April Tag replay")
            aprilTagReplay2.close()
            break

def onRelocalizeExtendedOutput(output, frameSet):
    global args, relocalizedFrameTime, relocalizedFramePose, relocalizeReplay1, relocalizeReplay2, pairedFrameTime, state
    #print(output.status)
    if not output.status == spectacularAI.TrackingStatus.TRACKING: return # not tracking yet

    for frame in frameSet:
        if not frame.image: continue
        if not pairedFrameTime:
            if not relocalizedFrameTime:
                relocalizedFrameTime = output.pose.time
                print("Tracking started with Relocalization at: {}".format(relocalizedFrameTime))
            # print("Close relocalize replay")
            relocalizeReplay1.close()
            break
        elif state == State.FindRelocalizePose and output.pose.time >= pairedFrameTime:
            if relocalizedFramePose is None:
                relocalizedFramePose = frame.cameraPose.pose.asMatrix()
                print("Relocalized pose at: {}\n{}".format(output.pose.time, relocalizedFramePose))
            # print("Close relocalize replay")
            relocalizeReplay2.close()
            break

def onAprilTagOutput(output):
    if not output.map.keyFrames: return # empty map

    # Find latest keyframe
    keyFrameId = max(output.map.keyFrames.keys())
    keyFrame = output.map.keyFrames[keyFrameId]
    frameSet = keyFrame.frameSet

    frame = frameSet.primaryFrame
    if frame is None or frame.image is None: return
    if frame.visualMarkers is None or frame.visualMarkers == []: return
    marker = frame.visualMarkers[0]
    if not marker.hasPose: 
        raise Exception("Found April Tag but it's not in the tags.json file")

def replayAprilTags():
    global args, aprilTagReplay1, aprilTagReplay2, state
    print("[Replay April Tags] {}".format(state))

    configInternal = {
        "aprilTagPath": args.dataFolder + "/tags.json",
        "extendParameterSets" : ["april-tags"]
    }

    replay = spectacularAI.Replay(args.dataFolder, onAprilTagOutput, configuration=configInternal)
    replay.setExtendedOutputCallback(onAprilTagExtendedOutput)
    
    if state == State.FindAprilTagTime:
        aprilTagReplay1 = replay
    else:
        aprilTagReplay2 = replay
    
    replay.runReplay()


def replayRelocalize():
    global args, relocalizeReplay1, relocalizeReplay2, state
    # print("[Replay Relocalize] {}".format(args))

    configInternal = {
        "mapLoadPath": args.dataFolder + "/map.bin",
        "extendParameterSets" : ["relocalization"]
    }

    replay = spectacularAI.Replay(args.dataFolder, configuration=configInternal)
    replay.setExtendedOutputCallback(onRelocalizeExtendedOutput)
 
    if state == State.FindRelocalizeTime:
        relocalizeReplay1 = replay
    else:
        relocalizeReplay2 = replay
 
    replay.runReplay()


if __name__ == '__main__':
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("dataFolder", nargs="?", help="Folder containing the recorded session and tags.json", default="data")
    p.add_argument("--preview", help="Show latest primary image as a preview", action="store_true")
    args =  p.parse_args()

    state = State.FindAprilTagTime
    print("Finding first April Tag frame time...")
    aprilTagThread = threading.Thread(target=replayAprilTags)
    aprilTagThread.start()
    while aprilTagFrameTime is None and aprilTagThread.is_alive():
        time.sleep(0.1)
    if aprilTagFrameTime is None:
        raise Exception("No valid frame found in April Tags replay")

    print("Finding first relocalized frame time...")
    state = State.FindRelocalizeTime
    relocalizeThread = threading.Thread(target=replayRelocalize)
    relocalizeThread.start()
    while relocalizedFrameTime is None and relocalizeThread.is_alive():
        time.sleep(0.1)
    if relocalizedFrameTime is None:
        raise Exception("No valid frame found in Relocalize replay")
    
    pairedFrameTime = max(aprilTagFrameTime, relocalizedFrameTime)
    print("Paired frame time: {}".format(pairedFrameTime))

    print("Finding paired time in April Tags replay...")
    state = State.FindAprilTagPose
    aprilTagThread = threading.Thread(target=replayAprilTags)
    aprilTagThread.start()
    while aprilTagFramePose is None and aprilTagThread.is_alive():
        time.sleep(0.1)
    if aprilTagFramePose is None:
        raise Exception("Paired frame not found in April Tags replay")

    print("Finding paired time in Relocalize replay...")
    state = State.FindRelocalizePose
    relocalizeThread = threading.Thread(target=replayRelocalize)
    relocalizeThread.start()
    while relocalizedFramePose is None and relocalizeThread.is_alive():
        time.sleep(0.1)
    if relocalizedFramePose is None:
        raise Exception("Paired frame not found in Relocalize replay")

    print("done")
    os._exit(0)

    