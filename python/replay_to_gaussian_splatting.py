#!/usr/bin/env python
#
# Replay existing session and convert output to format used by gaussian splatting method
#
# Use output with: https://github.com/wanmeihuali/taichi_3d_gaussian_splatting

import argparse
import spectacularAI
import cv2
import json
import os
import shutil
import math
import numpy as np
import pandas as pd


parser = argparse.ArgumentParser()
parser.add_argument("input", help="Path to folder with session to process")
parser.add_argument("output", help="Output folder, this should be 'data/' folder under taichi_3d_gaussian_splatting")
parser.add_argument("name", help="Session name", default="splatting_test")
parser.add_argument("--preview", help="Show latest primary image as a preview", action="store_true")
args = parser.parse_args()


# Globals
savedKeyFrames = {}
pointClouds = {}
frameWidth = -1
frameHeight = -1
intrinsics = None


def sharpness(path):
	img = cv2.imread(path)
	img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	return cv2.Laplacian(img, cv2.CV_64F).var()


def onMappingOutput(output):
    global savedKeyFrames
    global pointClouds
    global frameWidth
    global frameHeight
    global intrinsics

    if not output.finalMap:
        # New frames, let's save the images to disk
        for frameId in output.updatedKeyFrames:
            keyFrame = output.map.keyFrames.get(frameId)
            if not keyFrame or savedKeyFrames.get(frameId):
                continue
            savedKeyFrames[frameId] = True
            frameSet = keyFrame.frameSet
            if not frameSet.rgbFrame or not frameSet.rgbFrame.image:
                continue

            pointClouds[frameId] = np.copy(keyFrame.pointCloud.getPositionData())

            if frameWidth < 0:
                frameWidth = frameSet.rgbFrame.image.getWidth()
                frameHeight = frameSet.rgbFrame.image.getHeight()

            undistortedFrame = frameSet.getUndistortedFrame(frameSet.rgbFrame)
            if intrinsics is None: intrinsics = undistortedFrame.cameraPose.camera.getIntrinsicMatrix()
            img = undistortedFrame.image.toArray()
            bgrImage = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            fileName = f"{args.output}/{args.name}/tmp/frame_{frameId:05}.png"
            cv2.imwrite(fileName, bgrImage)
            if args.preview:
                cv2.imshow("Frame", bgrImage)
                cv2.setWindowTitle("Frame", "Frame #{}".format(frameId))
                cv2.waitKey(1)

    else:
        # Final optimized poses
        trainingFrames = []
        validationFrames = []
        globalPointCloud = []
        index = 0

        for frameId in output.map.keyFrames:
            # Image data
            keyFrame = output.map.keyFrames.get(frameId)
            oldImgName = f"{args.output}/{args.name}/tmp/frame_{frameId:05}.png"
            newImgName = f"{args.output}/{args.name}/images/frame_{index:05}.png"
            os.rename(oldImgName, newImgName)
            cameraPose = keyFrame.frameSet.rgbFrame.cameraPose

            # Camera data
            frame = {
                "image_path": f"data/{args.name}/images/frame_{index:05}.png",
                "T_pointcloud_camera": cameraPose.getCameraToWorldMatrix().tolist(), # 4x4 matrix, the transformation matrix from camera coordinate to point cloud coordinate
                "camera_intrinsics": intrinsics.tolist(), # 3x3 matrix, the camera intrinsics matrix K
                "camera_height": frameHeight, # image height, in pixel
                "camera_width": frameWidth, # image width, in pixel
                "camera_id": index # camera id, not used
            }
            if (index + 3) % 7 == 0:
                validationFrames.append(frame)
            else:
                trainingFrames.append(frame)

            # Pointcloud data
            pc = np.vstack((pointClouds[frameId].T, np.ones((1, pointClouds[frameId].shape[0]))))
            pc = (cameraPose.getCameraToWorldMatrix() @ pc)[:3, :].T
            globalPointCloud.extend(pc)

            index += 1

        # Save files
        point_cloud_df = pd.DataFrame(np.array(globalPointCloud), columns=["x", "y", "z"])
        point_cloud_df.to_parquet(f"{args.output}//{args.name}/point_cloud.parquet")

        # print(trainingFrames)

        with open(f"{args.output}/{args.name}/train.json", "w") as outFile:
            json.dump(trainingFrames, outFile, indent=2)

        with open(f"{args.output}/{args.name}/val.json", "w") as outFile:
            json.dump(validationFrames, outFile, indent=2)


def main():
    os.makedirs(f"{args.output}/{args.name}/images", exist_ok=True)
    os.makedirs(f"{args.output}/{args.name}/tmp", exist_ok=True)

    print("Processing")
    replay = spectacularAI.Replay(args.input, mapperCallback = onMappingOutput, configuration = {
        "keyframeDecisionDistanceThreshold": 0.01
    })

    replay.runReplay()

    shutil.rmtree(f"{args.output}/{args.name}/tmp")

    print("Done!")
    print("")
    print("You should use following paths in taichi_3d_gaussian_splatting config file:")
    print(f"pointcloud-parquet-path: 'data/{args.name}/point_cloud.parquet'")
    print(f"summary-writer-log-dir: data/{args.name}/logs")
    print(f"output-model-dir: data/{args.name}/output")
    print(f"train-dataset-json-path: 'data/{args.name}/train.json'")
    print(f"val-dataset-json-path: 'data/{args.name}/val.json'")

if __name__ == '__main__':
    main()
