import argparse
import spectacularAI
import cv2

frameTime = None
aprilTagFramePose = None
relocalizedFramePose = None
args = None
replay = None

def onAprilTagExtendedOutput(output, frameSet):
    global args, frameTime, relocalizedFramePose
    if (relocalizedFramePose is not None): return # already found
    # print(output.status)

    if not output.status == spectacularAI.TrackingStatus.TRACKING: return # not tracking yet

    for frame in frameSet:
        if not frame.image: continue
        aprilTagFramePose = frame.cameraPose.pose.asMatrix()
        frameTime = output.pose.time
        print("Tracking started with April Tag at: {}\n{}".format(frameTime, aprilTagFramePose))
        replay.close() # stop replay
        break

def onRelocalizeExtendedOutput(output, frameSet):
    global args, frameTime, aprilTagFramePose
    if (aprilTagFramePose is not None): return # already found
    print(output.status)

    if not output.status == spectacularAI.TrackingStatus.TRACKING: return # not tracking yet

    for frame in frameSet:
        if not frame.image: continue
        relocalizedFramePose = frame.cameraPose.pose.asMatrix()
        # print(dir(aprilTagFramePose))
        frameTime = output.pose.time
        print("Tracking started with Relocalization at: {}\n{}".format(frameTime, relocalizedFramePose))
        replay.close() # stop replay
        break

def onAprilTagOutput(output):
    global args, frameTime, aprilTagFramePose

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
    
# def onRelocalizeOutput(output):
    # nothing here

def replayAprilTags():
    global args, replay

    print("[Replay April Tags] {}".format(args))

    configInternal = {
        "aprilTagPath": args.dataFolder + "/tags.json",
        "extendParameterSets" : ["april-tags"]
    }

    replay = spectacularAI.Replay(args.dataFolder, onAprilTagOutput, configuration=configInternal)
    replay.setExtendedOutputCallback(onAprilTagExtendedOutput)
    replay.runReplay()

def replayRelocalize():
    global args, replay

    print("[Replay Relocalize] {}".format(args))

    configInternal = {
        "mapLoadPath": args.dataFolder + "/tags.json",
        "extendParameterSets" : ["relocalization"]
    }

    replay = spectacularAI.Replay(args.dataFolder, configuration=configInternal)
    replay.setExtendedOutputCallback(onRelocalizeExtendedOutput)
    replay.runReplay()

if __name__ == '__main__':
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("dataFolder", nargs="?", help="Folder containing the recorded session and tags.json", default="data")
    p.add_argument("--preview", help="Show latest primary image as a preview", action="store_true")
    args =  p.parse_args()

    # replayAprilTags()
    # if not frameTime:
    #     raise Exception("No valid frame found in April Tags replay")

    replayRelocalize()
    if not frameTime:
        raise Exception("No valid frame found in Relocalize replay")

    print("done")
    