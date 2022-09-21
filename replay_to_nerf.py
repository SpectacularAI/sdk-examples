#!/usr/bin/env python
#
# Replay existing session and convert output to format used by instant-ngp
#
# Use output with: https://github.com/NVlabs/instant-ngp

import argparse
import spectacularAI
import cv2
import json
import os
import shutil
import math
import numpy as np


parser = argparse.ArgumentParser()
parser.add_argument("input", help="Path to folder with session to process")
parser.add_argument("output", help="Path to output folder")
parser.add_argument("--scale", help="Scene scale, exponent of 2", type=int, default=128)
parser.add_argument("--preview", help="Show latest primary image as a preview", action="store_true")
args = parser.parse_args()

# Globals
savedKeyFrames = {}
frameWidth = -1
frameHeight = -1
intrinsics = None

TRANSFORM_CAM = np.array([
 [1,0,0,0],
 [0,-1,0,0],
 [0,0,-1,0],
 [0,0,0,1],
])

TRANSFORM_WORLD = np.array([
 [0,1,0,0],
 [-1,0,0,0],
 [0,0,1,0],
 [0,0,0,1],
])


def closestPointBetweenTwoLines(oa, da, ob, db):
    normal = np.cross(da, db)
    denom = np.linalg.norm(normal)**2
    t = ob - oa
    ta = np.linalg.det([t, db, normal]) / (denom + 1e-10)
    tb = np.linalg.det([t, da, normal]) / (denom + 1e-10)
    if ta > 0: ta = 0
    if tb > 0: tb = 0
    return ((oa + ta * da + ob + tb * db) * 0.5, denom)


def resizeToUnitCube(frames):
    weight = 0.0
    centerPos = np.array([0.0, 0.0, 0.0])
    for f in frames:
        mf = f["transform_matrix"][0:3,:]
        for g in frames:
            mg = g["transform_matrix"][0:3,:]
            p, w = closestPointBetweenTwoLines(mf[:,3], mf[:,2], mg[:,3], mg[:,2])
            if w > 0.00001:
                centerPos += p * w
                weight += w
    if weight > 0.0: centerPos /= weight

    scale = 0.
    for f in frames:
        f["transform_matrix"][0:3,3] -= centerPos
        scale += np.linalg.norm(f["transform_matrix"][0:3,3])

    scale = 4.0 / (scale / len(frames))
    for f in frames: f["transform_matrix"][0:3,3] *= scale


def sharpness(path):
	img = cv2.imread(path)
	img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	return cv2.Laplacian(img, cv2.CV_64F).var()


def onMappingOutput(output):
    global savedKeyFrames
    global frameWidth
    global frameHeight
    global intrinsics

    if not output.finalMap:
        # New frames, let's save the images to disk
        for frameId in output.updatedKeyFrames:
            keyFrame = output.map.keyFrames.get(frameId)
            if not keyFrame or savedKeyFrames.get(keyFrame):
                continue
            savedKeyFrames[keyFrame] = True
            frameSet = keyFrame.frameSet
            if not frameSet.rgbFrame or not frameSet.rgbFrame.image:
                continue

            if frameWidth < 0:
                frameWidth = frameSet.rgbFrame.image.getWidth()
                frameHeight = frameSet.rgbFrame.image.getHeight()

            undistortedFrame = frameSet.getUndistortedFrame(frameSet.rgbFrame)
            if intrinsics is None: intrinsics = undistortedFrame.cameraPose.camera.getIntrinsicMatrix()
            img = undistortedFrame.image.toArray()
            bgrImage = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            fileName = args.output + "/tmp/frame_" + f'{frameId:05}' + ".png"
            cv2.imwrite(fileName, bgrImage)
            if args.preview:
                cv2.imshow("Frame", bgrImage)
                cv2.setWindowTitle("Frame", "Frame #{}".format(frameId))
                cv2.waitKey(1)
    else:
        # Final optimized poses
        frames = []
        index = 0

        up = np.zeros(3)
        for frameId in output.map.keyFrames:
            keyFrame = output.map.keyFrames.get(frameId)
            oldImgName = args.output + "/tmp/frame_" + f'{frameId:05}' + ".png"
            newImgName = args.output + "/images/frame_" + f'{index:05}' + ".png"
            os.rename(oldImgName, newImgName)
            cameraPose = keyFrame.frameSet.rgbFrame.cameraPose

            # Converts Spectacular AI camera to coordinate system used by instant-ngp
            cameraToWorld = np.matmul(TRANSFORM_WORLD, np.matmul(cameraPose.getCameraToWorldMatrix(), TRANSFORM_CAM))
            up += cameraToWorld[0:3,1]
            frame = {
                "file_path": "images/frame_" + f'{index:05}' + ".png",
                "sharpness": sharpness(newImgName),
                "transform_matrix": cameraToWorld
            }
            frames.append(frame)
            index += 1

        resizeToUnitCube(frames)

        for f in frames: f["transform_matrix"] = f["transform_matrix"].tolist()

        if frameWidth < 0 or frameHeight < 0: raise Exception("Unable get image dimensions, zero images received?")

        fl_x = intrinsics[0][0]
        fl_y = intrinsics[1][1]
        cx = intrinsics[0][2]
        cy = intrinsics[1][2]
        angle_x = math.atan(frameWidth / (fl_x * 2)) * 2
        angle_y = math.atan(frameHeight / (fl_y * 2)) * 2

        transformationsJson = {
            "camera_angle_x": angle_x,
            "camera_angle_y": angle_y,
            "fl_x": fl_x,
            "fl_y": fl_y,
            "k1": 0.0,
            "k2": 0.0,
            "p1": 0.0,
            "p2": 0.0,
            "cx": cx,
            "cy": cy,
            "w": frameWidth,
            "h": frameHeight,
            "aabb_scale": args.scale,
            "frames": frames
        }

        with open(args.output + "/transformations.json", "w") as outFile:
            json.dump(transformationsJson, outFile, indent=2)


def main():
    os.makedirs(args.output + "/images", exist_ok=True)
    os.makedirs(args.output + "/tmp", exist_ok=True)

    print("Processing")
    replay = spectacularAI.Replay(args.input, onMappingOutput, config = {
        "globalBABeforeSave": True,              # Refine final map poses using bundle adjustment
        "maxMapSize": 0,                         # Unlimited map size
        "keyframeDecisionDistanceThreshold": 0.1 # Minimum distance between keyframes
    })

    replay.runReplay()

    shutil.rmtree(args.output + "/tmp")

    print("Done!")
    print("")
    print("You can now run instant-ngp nerfs using following command:")
    print("")
    print("    ./build/testbed --mode nerf --scene {}/transformations.json".format(args.output))


if __name__ == '__main__':
    main()
