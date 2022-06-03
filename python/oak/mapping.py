import argparse
import spectacularAI
import cv2
import json
import os
import depthai

p = argparse.ArgumentParser(__doc__)
p.add_argument("--dataFolder", help="Folder containing the recorded session for mapping")
p.add_argument("--outputFolder", help="Output folder for key frame images and their poses ", default="output")
p.add_argument("--preview", help="Show latest primary image as a preview", action="store_true")
args =  p.parse_args()

# KeyFrames for which we've already saved the image
savedKeyFrames = {}

running = True

def saveAsPng(outputFolder, frameId, cameraName, frame):
    if not frame or not frame.image: return
    fileName = outputFolder + "/" + cameraName + "_" + f'{frameId:05}' + ".png"
    cv2.imwrite(fileName, cv2.cvtColor(frame.image.toArray(), cv2.COLOR_RGB2BGR))


def onMappingOutput(output):
    global running
    global savedKeyFrames
    if output.finalMap:
        # Final optimized poses, let's save them to jsonl file
        with open(args.outputFolder + "/poses.jsonl", "w") as outFile:
            for frameId in output.map.keyFrames:
                keyFrame = output.map.keyFrames.get(frameId)
                outputJson = {
                    "frameId": frameId,
                    "poses": {}
                }
                frameSet = keyFrame.frameSet
                if frameSet.primaryFrame: outputJson["poses"]["primary"] = frameSet.primaryFrame.cameraPose.getCameraToWorldMatrix().tolist()
                if frameSet.secondaryFrame: outputJson["poses"]["secondary"] = frameSet.secondaryFrame.cameraPose.getCameraToWorldMatrix().tolist()
                if frameSet.rgbFrame: outputJson["poses"]["rgb"] = frameSet.rgbFrame.cameraPose.getCameraToWorldMatrix().tolist()
                if frameSet.depthFrame: outputJson["poses"]["depth"] = frameSet.depthFrame.cameraPose.getCameraToWorldMatrix().tolist()
                outFile.write(json.dumps(outputJson) + "\n")
    else:
        # New frames, let's save the images to disk
        for frameId in output.updatedKeyFrames:
            keyFrame = output.map.keyFrames.get(frameId)
            if not keyFrame or savedKeyFrames.get(keyFrame): continue
            savedKeyFrames[keyFrame] = True
            frameSet = keyFrame.frameSet
            if not frameSet: continue
            saveAsPng(args.outputFolder, frameId, "primary", frameSet.primaryFrame)
            saveAsPng(args.outputFolder, frameId, "secondary", frameSet.secondaryFrame)
            saveAsPng(args.outputFolder, frameId, "rgb", frameSet.rgbFrame)
            saveAsPng(args.outputFolder, frameId, "depth", frameSet.depthFrame)
            if args.preview and frameSet.primaryFrame and frameSet.primaryFrame.image:
                cv2.imshow("Primary camera", cv2.cvtColor(frameSet.primaryFrame.image.toArray(), cv2.COLOR_RGB2BGR))
                cv2.setWindowTitle("Primary camera", "Primary camera #{} (Q or ESC to exit)".format(frameId))
                key = cv2.waitKey(1)
                if key == 113 or key == 27: # q or ESC
                    running = False


if not os.path.isdir(args.outputFolder): os.makedirs(args.outputFolder)

if args.dataFolder:
    print("Starting replay")
    replay = spectacularAI.Replay(args.dataFolder, onMappingOutput)
    replay.startReplay()
else:
    print("Starting OAK-D device")
    pipeline = depthai.Pipeline()
    config = spectacularAI.depthai.Configuration()
    vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, config, onMappingOutput)
    with depthai.Device(pipeline) as device, \
        vio_pipeline.startSession(device) as vio_session:
        while running:
            out = vio_session.waitForOutput()

