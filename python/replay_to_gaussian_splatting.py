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
import numpy as np
import pandas as pd
from scipy.spatial import KDTree

parser = argparse.ArgumentParser()
parser.add_argument("input", help="Path to folder with session to process")
parser.add_argument("output", help="Output folder, this should be 'data/' folder under taichi_3d_gaussian_splatting")
parser.add_argument("--name", help="Session name", default="splatting_test")
parser.add_argument("--cell_size", help="Point cloud decimation cell size", type=float, default=0.025)
parser.add_argument("--key_frame_distance", help="Minimum distance between keyframes (meters)", type=float, default=0.05)
parser.add_argument('--device_preset', choices=['none', 'k4a', 'realsense'], default='none')
parser.add_argument('--slow', action='store_true')
parser.add_argument("--preview", help="Show latest primary image as a preview", action="store_true")
args = parser.parse_args()

def interpolate_missing_properties(df_source, df_query, k_nearest=3):
    xyz = list('xyz')

    tree = KDTree(df_source[xyz].values)
    _, ii = tree.query(df_query[xyz], k=k_nearest)
    n = df_query.shape[0]

    df_result = pd.DataFrame(0, index=range(n), columns=df_source.columns)
    df_result[xyz] = df_query[xyz]
    other_cols = [c for c in df_source.columns if c not in xyz]

    for i in range(n):
        m = df_source.loc[ii[i].tolist(), other_cols].mean(axis=0)
        df_result.loc[i, other_cols] = m

    return df_result

def voxel_decimate(df, cell_size):
    def grouping_function(row):
        return tuple([round(row[c] / cell_size) for c in 'xyz'])
    df['voxel_index'] = df.apply(grouping_function, axis=1)
    grouped = df.groupby('voxel_index')
    return grouped.first().reset_index()

# Globals
savedKeyFrames = {}
pointClouds = {}
frameWidth = -1
frameHeight = -1
intrinsics = None

def blurScore(path):
    image = cv2.imread(path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    f_transform = np.fft.fft2(gray)
    f_transform_shifted = np.fft.fftshift(f_transform)
    magnitude_spectrum = np.abs(f_transform_shifted)
    return np.percentile(magnitude_spectrum, 95)


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

            pointClouds[frameId] = (np.copy(keyFrame.pointCloud.getPositionData()), np.copy(keyFrame.pointCloud.getRGB24Data()))

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

        blurryImages = {}
        imageSharpness = []
        for frameId in output.map.keyFrames:
            imageSharpness.append((frameId, blurScore(f"{args.output}/{args.name}/tmp/frame_{frameId:05}.png")))

        # Look two images forward and two backwards, if current frame is blurriest, don't use it
        for i in range(len(imageSharpness)):
            if i + 2 > len(imageSharpness): break
            group = [imageSharpness[j+i] for j in range(-2,2)]
            group.sort(key=lambda x : x[1])
            cur = imageSharpness[i][0]
            if group[0][0] == cur:
                blurryImages[cur] = True

        trainingFrames = []
        validationFrames = []
        globalPointCloud = []
        index = 0
        for frameId in output.map.keyFrames:
            if blurryImages.get(frameId): continue # Skip blurry images

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
            posData, colorData = pointClouds[frameId]
            pc = np.vstack((posData.T, np.ones((1, posData.shape[0]))))
            pc = (cameraPose.getCameraToWorldMatrix() @ pc)[:3, :].T
            pc = np.hstack((pc, colorData))
            globalPointCloud.extend(pc)

            index += 1

        # Save files
        point_cloud_df = pd.DataFrame(np.array(globalPointCloud), columns=list('xyzrgb'))
        # point_cloud_df.to_csv(f"{args.output}/{args.name}/points.dense.csv", index=False)

        # drop uncolored points
        colored_point_cloud_df = point_cloud_df.loc[point_cloud_df[list('rgb')].max(axis=1) > 0].reset_index()
        sparse_point_cloud_df = pd.read_csv(f"{args.output}/{args.name}/points.sparse.csv", usecols=list('xyz'))

        sparse_colored_point_cloud_df = interpolate_missing_properties(colored_point_cloud_df, sparse_point_cloud_df)
        decimated_df = voxel_decimate(colored_point_cloud_df, args.cell_size)
        merged_df = pd.concat([sparse_colored_point_cloud_df, decimated_df])
        # merged_df.to_csv(f"{args.output}/{args.name}/points.merged-decimated.csv", index=False)
        merged_df.to_parquet(f"{args.output}/{args.name}/point_cloud.parquet")

        with open(f"{args.output}/{args.name}/train.json", "w") as outFile:
            json.dump(trainingFrames, outFile, indent=2)

        with open(f"{args.output}/{args.name}/val.json", "w") as outFile:
            json.dump(validationFrames, outFile, indent=2)


def main():
    os.makedirs(f"{args.output}/{args.name}/images", exist_ok=True)
    tmp_dir = f"{args.output}/{args.name}/tmp"
    tmp_input = f"{tmp_dir}/input"
    os.makedirs(tmp_input, exist_ok=True)
    shutil.rmtree(tmp_input)
    from distutils.dir_util import copy_tree
    copy_tree(args.input, tmp_input)

    parameter_sets = ['wrapper-base', args.device_preset, 'offline-base']
    if args.device_preset == 'k4a':
        parameter_sets.extend(['icp', 'offline-icp'])
    elif args.device_preset == 'realsense':
        parameter_sets.extend(['icp', 'realsense-icp', 'offline-icp'])

    with open(tmp_input + "/vio_config.yaml", 'wt') as f:
        base_params = 'parameterSets: %s' % json.dumps(parameter_sets)
        f.write(base_params + '\n')
        print(base_params)

    config = {
        "maxMapSize": 0,
        "keyframeDecisionDistanceThreshold": args.key_frame_distance,
        "maxKeypoints": 2000,
        "mergeMultiLevelPointsThresholdPixels": 1,
        "orbRelativeMaskRadius": 0.01,
        "mapSavePath": f"{args.output}/{args.name}/points.sparse.csv"
    }

    if args.slow:
        ext = {
            "keyframeCandidateInterval": 2,
            "globalBABeforeSave": True,
            "globalBAAfterLoopClosure": True,
            "mapPointCullingMinObservationCount": 1 # most dense SfM point cloud
        }
        for k, v in ext.items(): config[k] = v

    print(config)
    replay = spectacularAI.Replay(tmp_input, mapperCallback = onMappingOutput, configuration = config)

    replay.runReplay()

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
