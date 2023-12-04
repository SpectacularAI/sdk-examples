#!/usr/bin/env python
"""
Post-process data in Spectacular AI format and convert it to input
for NeRF or Gaussian Splatting methods.
"""

import argparse
import spectacularAI
import cv2
import json
import os
import shutil
import numpy as np
import pandas as pd
from scipy.spatial import KDTree

parser = argparse.ArgumentParser(__doc__)
parser.add_argument("input", help="Path to folder with session to process")
parser.add_argument("output", help="Output folder")
parser.add_argument('--format', choices=['taichi', 'nerfstudio'], default='nerfstudio', help='Output format')
parser.add_argument("--cell_size", help="Point cloud decimation cell size", type=float, default=0.025)
parser.add_argument("--distance_quantile", help="Max point distance filter quantile (0 = disabled)", type=float, default=0.99)
parser.add_argument("--key_frame_distance", help="Minimum distance between keyframes (meters)", type=float, default=0.05)
parser.add_argument('--no_icp', action='store_true')
parser.add_argument('--device_preset', choices=['none', 'oak-d', 'k4a', 'realsense', 'android-tof', 'ios-tof'], help="Automatically detected in most cases")
parser.add_argument('--fast', action='store_true', help='Fast but lower quality settings')
parser.add_argument('--mono', action='store_true', help='Monocular mode: disable ToF and stereo data')
parser.add_argument('--image_format', type=str, default='jpg', help="Color image format (use 'png' for top quality)")
parser.add_argument("--preview", help="Show latest primary image as a preview", action="store_true")
parser.add_argument("--preview3d", help="Show 3D visualization", action="store_true")
args = parser.parse_args()

useMono = None

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

def exclude_points(df_source, df_exclude, radius):
    xyz = list('xyz')
    tree = KDTree(df_exclude[xyz].values)
    ii = tree.query_ball_point(df_source[xyz], r=radius, return_length=True)
    mask = [l == 0 for l in ii]
    df_result = df_source.iloc[mask]
    return df_result

def voxel_decimate(df, cell_size):
    def grouping_function(row):
        return tuple([round(row[c] / cell_size) for c in 'xyz'])
    grouped = df.assign(voxel_index=df.apply(grouping_function, axis=1)).groupby('voxel_index')
    return grouped.first().reset_index()[[c for c in df.columns if c != 'voxel_index']]

def convert_json_taichi_to_nerfstudio(d):
    def transform_camera(c):
        convention_change = np.array([
            [1, 0, 0, 0],
            [0,-1, 0, 0],
            [0, 0,-1, 0],
            [0, 0, 0, 1]
        ])
        return (np.array(c) @ convention_change).tolist()

    by_camera = {}
    for c in d:
        k = c['camera_intrinsics']
        params = {
            "fl_x": k[0][0],
            "fl_y": k[1][1],
            "k1": 0,
            "k2": 0,
            "p1": 0,
            "p2": 0,
            "cx": k[0][2],
            "cy": k[1][2],
            "w": c['camera_width'],
            "h": c['camera_height'],
            "aabb_scale": 16,
            'frames': []
        }
        cam_id = json.dumps(params, sort_keys=True)
        if cam_id not in by_camera:
            by_camera[cam_id] = params

        converted = {
            'file_path': "./images/" + c['image_path'].split('/')[-1],
            "transform_matrix": transform_camera(c['T_pointcloud_camera'])
        }
        if 'depth_image_path' in c:
            converted['depth_file_path'] = "./images/" + c['depth_image_path'].split('/')[-1]

        by_camera[cam_id]['frames'].append(converted)

    if len(by_camera) != 1:
        raise RuntimeError("unexpected number of cameras")

    key, value = list(by_camera.items())[0]
    return value

# TODO: don't use "Taichi" as the intermediate format
def convert_json_taichi_to_colmap(pose_data, points_df, nerfstudio_fake_obs=True):
    from scipy.spatial.transform import Rotation as R

    images = []
    cameras = []
    camera_id = 0
    for image_id, c in enumerate(pose_data):
        k = c['camera_intrinsics']
        mat = np.linalg.inv(np.array(c['T_pointcloud_camera']))
        qx,qy,qz,qw = R.from_matrix(mat[:3,:3]).as_quat()
        q = [qw, qx, qy, qz]
        p = list(mat[:3, 3])
        images.append([image_id] + list(q) + list(p) + [camera_id, os.path.split(c['image_path'])[-1]])

        points = []
        if nerfstudio_fake_obs:
            points = [100,100,0,200,200,1] # NeRFstudio loader will crash without this

        images.append(points)

        # TODO: variable intrinsics
        if len(cameras) == 0:
            cameras = [[
                camera_id,
                'PINHOLE',
                c['camera_width'],
                c['camera_height'],
                k[0][0],
                k[1][1],
                k[0][2],
                k[1][2]
            ]]

    points = []
    for point_id, row in points_df.iterrows():
        point = [
            point_id,
            row['x'],
            row['y'],
            row['z'],
            round(row['r']),
            round(row['g']),
            round(row['b'])
        ]

        if nerfstudio_fake_obs:
            fake_err = 1
            img_id, point_id = 0, 0
            point.extend([fake_err, img_id, point_id])

        points.append(point)

    return points, images, cameras

# Globals
savedKeyFrames = {}
pointClouds = {}
frameWidth = -1
frameHeight = -1
intrinsics = None
visualizer = None

def blurScore(path):
    image = cv2.imread(path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    f_transform = np.fft.fft2(gray)
    f_transform_shifted = np.fft.fftshift(f_transform)
    magnitude_spectrum = np.abs(f_transform_shifted)
    return np.percentile(magnitude_spectrum, 95)

def post_process_point_clouds(globalPointCloud, sparse_point_cloud_df):
    # Save point clouds
    if len(globalPointCloud) == 0:
        # add fake (gray) colors
        merged_df = sparse_point_cloud_df
        for c in 'rgb': merged_df[c] = 128

    else:
        point_cloud_df = pd.DataFrame(np.array(globalPointCloud), columns=list('xyzrgb'))

        # drop uncolored points
        colored_point_cloud_df = point_cloud_df.loc[point_cloud_df[list('rgb')].max(axis=1) > 0].reset_index()

        filtered_point_cloud_df = exclude_points(colored_point_cloud_df, sparse_point_cloud_df, radius=args.cell_size)
        decimated_df = voxel_decimate(filtered_point_cloud_df, args.cell_size)
        sparse_colored_point_cloud_df = interpolate_missing_properties(colored_point_cloud_df, sparse_point_cloud_df)
        merged_df = pd.concat([sparse_colored_point_cloud_df, decimated_df])

    if args.distance_quantile > 0:
        dist2 = (merged_df[list('xyz')]**2).sum(axis=1).values
        MARGIN = 1.5
        max_dist2 = np.quantile(dist2, args.distance_quantile) * MARGIN**2
        print(f'filtering out points further than {np.sqrt(max_dist2)}m')
        merged_df = merged_df.iloc[dist2 < max_dist2]

    return merged_df

def onVioOutput(vioOutput):
    global visualizer
    if visualizer is not None:
        visualizer.onVioOutput(vioOutput.getCameraPose(0))

def onMappingOutput(output):
    global savedKeyFrames
    global pointClouds
    global frameWidth
    global frameHeight
    global intrinsics
    global visualizer
    global useMono

    if visualizer is not None:
        visualizer.onMappingOutput(output)

    if not output.finalMap:
        # New frames, let's save the images to disk
        for frameId in output.updatedKeyFrames:
            keyFrame = output.map.keyFrames.get(frameId)
            if not keyFrame or savedKeyFrames.get(frameId):
                continue
            savedKeyFrames[frameId] = True
            frameSet = keyFrame.frameSet
            targetFrame = frameSet.rgbFrame
            if not targetFrame: targetFrame = frameSet.primaryFrame
            if not targetFrame or not targetFrame.image: continue

            if keyFrame.pointCloud:
                pointClouds[frameId] = (
                    np.copy(keyFrame.pointCloud.getPositionData()),
                    np.copy(keyFrame.pointCloud.getRGB24Data()))

            if frameWidth < 0:
                frameWidth = targetFrame.image.getWidth()
                frameHeight = targetFrame.image.getHeight()

            undistortedFrame = frameSet.getUndistortedFrame(targetFrame)
            if intrinsics is None: intrinsics = undistortedFrame.cameraPose.camera.getIntrinsicMatrix()
            img = undistortedFrame.image.toArray()
            bgrImage = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            fileName = f"{args.output}/tmp/frame_{frameId:05}.{args.image_format}"
            cv2.imwrite(fileName, bgrImage)

            if frameSet.depthFrame.image is not None and not useMono:
                alignedDepth = frameSet.getAlignedDepthFrame(undistortedFrame)
                depthData = alignedDepth.image.toArray()
                depthFrameName = f"{args.output}/tmp/depth_{frameId:05}.png"
                cv2.imwrite(depthFrameName, depthData)

                DEPTH_PREVIEW = False
                if args.preview and DEPTH_PREVIEW:
                    DEPTH_COLOR_MAP_MIDPOINT_M = 2.0
                    visuDepth = np.log1p(depthData * alignedDepth.depthScale) / np.log1p(DEPTH_COLOR_MAP_MIDPOINT_M) * 0.5 * 256
                    cv2.imshow("Depth frame", cv2.applyColorMap(np.clip(visuDepth, 0, 255).astype(np.uint8), cv2.COLORMAP_JET))

            # TODO: move these visualizations to the main thread
            if args.preview:
                cv2.imshow("Frame", bgrImage)
                cv2.setWindowTitle("Frame", "Frame #{}".format(frameId))
                cv2.waitKey(1)

    else:
        # Final optimized poses

        blurryImages = {}
        imageSharpness = []
        for frameId in output.map.keyFrames:
            imageSharpness.append((frameId, blurScore(f"{args.output}/tmp/frame_{frameId:05}.{args.image_format}")))

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
        name = os.path.split(args.output)[-1]
        for frameId in output.map.keyFrames:
            if blurryImages.get(frameId): continue # Skip blurry images

            # Image data
            keyFrame = output.map.keyFrames.get(frameId)

            targetFrame = keyFrame.frameSet.rgbFrame
            if not targetFrame: targetFrame = keyFrame.frameSet.primaryFrame
            cameraPose = targetFrame.cameraPose

            # Camera data
            frame = {
                "image_path": f"data/{name}/images/frame_{index:05}.{args.image_format}",
                "T_pointcloud_camera": cameraPose.getCameraToWorldMatrix().tolist(), # 4x4 matrix, the transformation matrix from camera coordinate to point cloud coordinate
                "camera_intrinsics": intrinsics.tolist(), # 3x3 matrix, the camera intrinsics matrix K
                "camera_height": frameHeight, # image height, in pixel
                "camera_width": frameWidth, # image width, in pixel
                "camera_id": index # camera id, not used
            }

            oldImgName = f"{args.output}/tmp/frame_{frameId:05}.{args.image_format}"
            newImgName = f"{args.output}/images/frame_{index:05}.{args.image_format}"
            os.rename(oldImgName, newImgName)

            oldDepth = f"{args.output}/tmp/depth_{frameId:05}.png"
            newDepth = f"{args.output}/images/depth_{index:05}.png"
            if os.path.exists(oldDepth):
                os.rename(oldDepth, newDepth)
                frame['depth_image_path'] = f"data/{name}/images/depth_{index:05}.png"

            if (index + 3) % 7 == 0:
                validationFrames.append(frame)
            else:
                trainingFrames.append(frame)

            if frameId in pointClouds:
                # Pointcloud data
                posData, colorData = pointClouds[frameId]
                pc = np.vstack((posData.T, np.ones((1, posData.shape[0]))))
                pc = (cameraPose.getCameraToWorldMatrix() @ pc)[:3, :].T
                pc = np.hstack((pc, colorData))
                globalPointCloud.extend(pc)

            index += 1

        merged_df = post_process_point_clouds(
            globalPointCloud,
            pd.read_csv(f"{args.output}/points.sparse.csv", usecols=list('xyz')))

        if args.format == 'taichi':
            # merged_df.to_csv(f"{args.output}/points.merged-decimated.csv", index=False)
            merged_df.to_parquet(f"{args.output}/point_cloud.parquet")

            with open(f"{args.output}/train.json", "w") as outFile:
                json.dump(trainingFrames, outFile, indent=2, sort_keys=True)

            with open(f"{args.output}/val.json", "w") as outFile:
                json.dump(validationFrames, outFile, indent=2, sort_keys=True)
        elif args.format == 'nerfstudio':
            allFrames = trainingFrames + validationFrames
            with open(f"{args.output}/transforms.json", "w") as outFile:
                json.dump(convert_json_taichi_to_nerfstudio(allFrames), outFile, indent=2, sort_keys=True)

            # colmap text point format
            fake_colmap = f"{args.output}/colmap/sparse/0"
            os.makedirs(fake_colmap, exist_ok=True)

            c_points, c_images, c_cameras = convert_json_taichi_to_colmap(allFrames, merged_df, nerfstudio_fake_obs=True)

            def write_colmap_csv(data, fn):
                with open(fn, 'wt') as f:
                    for row in data:
                        f.write(' '.join([str(c) for c in row])+'\n')

            write_colmap_csv(c_points, f"{fake_colmap}/points3D.txt")
            write_colmap_csv(c_images, f"{fake_colmap}/images.txt")
            write_colmap_csv(c_cameras, f"{fake_colmap}/cameras.txt")

def copy_input_to_tmp_safe(input_dir, tmp_input):
    # also works if tmp dir is inside the input directory
    os.makedirs(tmp_input, exist_ok=True)
    shutil.rmtree(tmp_input)
    os.makedirs(tmp_input)
    for f in os.listdir(input_dir):
        full_fn = os.path.join(input_dir, f)
        if not os.path.isdir(full_fn): shutil.copy(full_fn, tmp_input)
        elif f.startswith("frames"): shutil.copytree(full_fn, f"{tmp_input}/{f}", dirs_exist_ok=True)

def detect_device_preset(input_dir):
    cameras = None
    calibrationJson = f"{input_dir}/calibration.json"
    if os.path.exists(calibrationJson):
        with open(calibrationJson) as f:
            calibration = json.load(f)
            if "cameras" in calibration:
                cameras = len(calibration["cameras"])
    device = None
    metadataJson = f"{input_dir}/metadata.json"
    if os.path.exists(metadataJson):
        with open(metadataJson) as f:
             metadata = json.load(f)
             if metadata.get("platform") == "ios":
                device = "ios-tof"
    if device == None:
        vioConfigYaml = f"{input_dir}/vio_config.yaml"
        if os.path.exists(vioConfigYaml):
            with open(vioConfigYaml) as file:
                for line in file:
                    if "parameterSets" in line:
                        if "oak-d" in line: device = "oak-d"
                        if "k4a" in line: device = "k4a"
                        if "realsense" in line: device = "realsense"
                    if device: break
    return (device, cameras)

def main():
    global visualizer
    global useMono

    os.makedirs(f"{args.output}/images", exist_ok=True)
    tmp_dir = f"{args.output}/tmp"
    tmp_input = f"{tmp_dir}/input"
    copy_input_to_tmp_safe(args.input, tmp_input)

    config = {
        "maxMapSize": 0,
        "useSlam": True,
        "passthroughColorImages": True,
        "keyframeDecisionDistanceThreshold": args.key_frame_distance,
        "icpVoxelSize": min(args.key_frame_distance, 0.1),
        "mapSavePath": f"{args.output}/points.sparse.csv"
    }

    device_preset, cameras = detect_device_preset(args.input)

    useMono = args.mono or (cameras != None and cameras == 1)

    if useMono: config['useStereo'] = False

    prefer_icp = not args.no_icp and not useMono
    parameter_sets = ['wrapper-base']

    if not args.fast:
        parameter_sets.append('offline-base')
        # remove these to further trade off speed for quality
        mid_q = {
            'maxKeypoints': 1000,
            'optimizerMaxIterations': 30
        }
        for k, v in mid_q.items(): config[k] = v

    if args.device_preset:
        device_preset = args.device_preset


    if device_preset: print(f"Selected device type: {device_preset}")
    else: print("Warning! Couldn't automatically detect device preset, to ensure best results suply one via --device_preset argument")

    if device_preset:
        parameter_sets.append(device_preset)

    if device_preset == 'k4a':
        if prefer_icp:
            parameter_sets.extend(['icp'])
            if not args.fast: parameter_sets.append('offline-icp')
    elif device_preset == 'realsense':
        if prefer_icp:
            parameter_sets.extend(['icp', 'realsense-icp'])
            if not args.fast: parameter_sets.append('offline-icp')
        config['stereoPointCloudStride'] = 15
    elif device_preset == 'oak-d':
        config['stereoPointCloudMinDepth'] = 0.5
        config['stereoPointCloudStride'] = 30

    if args.preview3d:
        from visualization.visualizer import Visualizer
        visualizer = Visualizer()

    with open(tmp_input + "/vio_config.yaml", 'wt') as f:
        base_params = 'parameterSets: %s' % json.dumps(parameter_sets)
        f.write(base_params + '\n')
        print(base_params)

    print(config)
    replay = spectacularAI.Replay(tmp_input, mapperCallback = onMappingOutput, configuration = config)
    replay.setOutputCallback(onVioOutput)

    if visualizer is None:
        replay.runReplay()
    else:
        replay.startReplay()
        visualizer.run()
        replay.close()

    try:
        shutil.rmtree(tmp_dir)
    except:
        print(f"Failed to clean temporary directory, you can delete these files manually, they are no longer required: {tmp_dir}")

    print("Done!\n")

    if args.format == 'taichi':
        name = os.path.split(args.output)[-1]
        print("You should use following paths in taichi_3d_gaussian_splatting config file:")
        print(f"pointcloud-parquet-path: 'data/{name}/point_cloud.parquet'")
        print(f"summary-writer-log-dir: data/{name}/logs")
        print(f"output-model-dir: data/{name}/output")
        print(f"train-dataset-json-path: 'data/{name}/train.json'")
        print(f"val-dataset-json-path: 'data/{name}/val.json'")
    elif args.format == 'nerfstudio':
        print(f'output written to {args.output}')

if __name__ == '__main__':
    main()
