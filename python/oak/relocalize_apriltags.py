import argparse
import spectacularAI
import cv2

frameTime = None
aprilTagFramePose = None
relocalizedFramePose = None

def onOutput(output):
    global frameTime
    if output.frameSet is not None:
        frameTime = output.frameSet.timestamp

def onAprilTagMappingOutput(output, frameSet):
    global frameTime, aprilTagFramePose
    
    for frame in frameSet:
        if args.preview and frame.image:
            cv2.imshow("Camera #{}".format(frame.index), cv2.cvtColor(frame.image.toArray(), cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)
    print(output.asJson())

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
        print("Warning: Found April Tag but it's not in the tags.json file")
        return
    
    frameTime = output.frameSet.timestamp
    aprilTagFramePose = frame.cameraPose.asMatrix()
    print("Found April Tag at timestamp [{}] with pose: {}".format(frame.timestamp, aprilTagFramePose))

if __name__ == '__main__':
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("dataFolder", help="Folder containing the recorded session and tags.json", default="data")
    p.add_argument("--preview", help="Show latest primary image as a preview", action="store_true")
    args =  p.parse_args()

    replay = spectacularAI.Replay(args.dataFolder)

    replay.setExtendedOutputCallback(onAprilTagMappingOutput)
    replay.runReplay()
