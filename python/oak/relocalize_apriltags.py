import argparse
import spectacularAI
import time
import threading
import os
import numpy as np
from enum import Enum
import json

class State(Enum):
    FindRelocalizeTime = 1
    FindAprilTagTime = 2
    FindRelocalizePose = 3

aprilTagFrameTime = None
aprilTagFramePose = None
relocalizedFrameTime = None
relocalizedFramePose = None
args = None
aprilTagReplay1 = None
relocalizeReplay1 = None
relocalizeReplay2 = None
state = None    

def onRelocalizeExtendedOutput(output, frameSet):
    global args, relocalizedFrameTime, relocalizedFramePose, relocalizeReplay1, relocalizeReplay2, pairedFrameTime, state
    if not output.status == spectacularAI.TrackingStatus.TRACKING: return # not tracking yet

    for frame in frameSet:
        if not frame.image: continue
        if state == State.FindRelocalizeTime:
            if not relocalizedFrameTime:
                relocalizedFrameTime = output.pose.time
                print("Tracking started with Relocalization at: {}".format(relocalizedFrameTime))
            # print("Close relocalize replay")
            relocalizeReplay1.close()
            break
        elif state == State.FindRelocalizePose and output.pose.time >= aprilTagFrameTime:
            if relocalizedFramePose is None:
                relocalizedFramePose = frame.cameraPose.pose.asMatrix()
                print("Camera to relocalized world at: {}\n{}".format(output.pose.time, relocalizedFramePose))
            # print("Close relocalize replay")
            relocalizeReplay2.close()
            break

def onAprilTagOutput(output):
    global args, aprilTagFrameTime, aprilTagFramePose, aprilTagReplay1, state
    
    if aprilTagFrameTime is not None:
        # print("Close April Tag replay")
        aprilTagReplay1.close()
        return

    if not state == State.FindAprilTagTime: return # not looking for April Tag time
    # if not output.status == spectacularAI.TrackingStatus.TRACKING: return # not tracking yet
    if not output.map.keyFrames: return # empty map

    # Find latest keyframe
    keyFrameId = max(output.map.keyFrames.keys())
    keyFrame = output.map.keyFrames[keyFrameId]
    frameSet = keyFrame.frameSet
    frame = frameSet.primaryFrame
    frameTime = frame.cameraPose.pose.time
    if not frameTime >= relocalizedFrameTime: return # not after relocalized time

    # find a frame with a marker
    if frame is None or frame.image is None: return
    if frame.visualMarkers is None or frame.visualMarkers == []: return
    marker = frame.visualMarkers[0]
    if not marker.hasPose: 
        raise Exception("Found April Tag but it's not in the tags.json file")
    
    aprilTagFrameTime = frameTime
    aprilTagFramePose = marker.pose.asMatrix()
    print("Camera to April Tag at: {}\n{}".format(frameTime, aprilTagFramePose))

def replayAprilTags():
    global args, aprilTagReplay1, aprilTagReplay2, state
    print("[Replay April Tags] {}".format(state))

    configInternal = {
        "aprilTagPath": args.dataFolder + "/tags.json",
        "extendParameterSets" : ["april-tags"]
    }

    aprilTagReplay1 = spectacularAI.Replay(args.dataFolder, onAprilTagOutput, configuration=configInternal)
    aprilTagReplay1.runReplay()


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

def compute_slam_to_dt(camera_to_slam, camera_to_tag, tag_to_dt):
    """
    Computes the transformation from the SLAM (relocalization) frame to the Digital Twin frame.

    Parameters:
        camera_to_slam (np.ndarray): 4x4 transformation matrix (T^(S)_(C)) representing
                                     the camera pose in the SLAM coordinate frame.
        camera_to_tag (np.ndarray): 4x4 transformation matrix (T^(C)_(A)) representing
                                     the AprilTag pose in the camera coordinate frame.
        tag_to_dt (np.ndarray): 4x4 transformation matrix (T^(D)_(A)) representing
                                the AprilTag pose in the Digital Twin coordinate frame.

    Returns:
        np.ndarray: 4x4 transformation matrix (T^(D)_(S)) that maps a point from the
                    SLAM coordinate frame to the Digital Twin coordinate frame.
    """
    # Compute the inverse of camera_to_tag (T^(C)_(A))
    tag_to_camera = np.linalg.inv(camera_to_tag)

    # Compute the transformation from SLAM to Digital Twin (T^(D)_(S))
    slam_to_dt = tag_to_dt @ tag_to_camera @ camera_to_slam

    return slam_to_dt

if __name__ == '__main__':
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("dataFolder", nargs="?", help="Folder containing the recorded session and tags.json", default="data")
    p.add_argument("--preview", help="Show latest primary image as a preview", action="store_true")
    args =  p.parse_args()

    print("Pose of first April Tag from tags.json:")
    with open(args.dataFolder + "/tags.json", 'r') as tags_file:
        tags_json = json.load(tags_file)
    aprilTagToWorld = np.array(tags_json[0]['tagToWorld'])
    print(aprilTagToWorld)

    print("Finding first relocalized frame time...")
    state = State.FindRelocalizeTime
    relocalizeThread = threading.Thread(target=replayRelocalize)
    relocalizeThread.start()
    while relocalizedFrameTime is None and relocalizeThread.is_alive():
        time.sleep(0.1)
    if relocalizedFrameTime is None:
        raise Exception("No valid frame found in Relocalize replay")

    state = State.FindAprilTagTime
    print("Finding April Tag frame time (after Relocalized time)...")
    aprilTagThread = threading.Thread(target=replayAprilTags)
    aprilTagThread.start()
    while aprilTagFrameTime is None and aprilTagThread.is_alive():
        time.sleep(0.1)
    if aprilTagFrameTime is None:
        raise Exception("No valid frame found in April Tags replay")
    
    print("Finding paired time in Relocalize replay...")
    state = State.FindRelocalizePose
    relocalizeThread = threading.Thread(target=replayRelocalize)
    relocalizeThread.start()
    while relocalizedFramePose is None and relocalizeThread.is_alive():
        time.sleep(0.1)
    if relocalizedFramePose is None:
        raise Exception("Paired frame not found in Relocalize replay")

    print("SLAM to Digital Twin transformation:")
    slam_to_dt = compute_slam_to_dt(relocalizedFramePose, 
                                    aprilTagFramePose, 
                                    aprilTagToWorld)
    print(slam_to_dt)

    slam_config_json = {
        "slamToUnity": slam_to_dt.tolist()
    }
    slam_config_path = args.dataFolder + "/slam_config.json"
    with open(slam_config_path, 'w') as slam_config_file:
        json.dump(slam_config_json, slam_config_file, indent=4)
    print("Wrote transformation to: {}".format(slam_config_path))
    os._exit(0)

    