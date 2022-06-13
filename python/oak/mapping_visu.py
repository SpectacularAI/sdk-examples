"""
Visualize 3D point cloud of the environment in real-time, or playback your recordings and view their 3D point cloud.
Press 'H' to view Open3D point cloud viewer options.

Requirements: pip install open3d
"""

import numpy as np
import spectacularAI
import threading
import open3d as o3d
import time
import depthai
import os

# Wrapper around Open3D point cloud, which helps updating its world pose.
class PointCloud:
    def __init__(self, keyFrame, voxelSize, colorOnly):
        self.camToWorld = np.identity(4)
        self.cloud = self.__getKeyFramePointCloud(keyFrame, voxelSize, colorOnly)

    def __getKeyFramePointCloud(self, keyFrame, voxelSize, colorOnly):
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(keyFrame.pointCloud.getPositionData())

        if keyFrame.pointCloud.hasColors():
            colors = keyFrame.pointCloud.getRGB24Data() * 1./255
            cloud.colors = o3d.utility.Vector3dVector(colors)

        if keyFrame.pointCloud.hasNormals():
            cloud.normals = o3d.utility.Vector3dVector(keyFrame.pointCloud.getNormalData())

        if cloud.has_colors() and colorOnly:
            # Filter points without color
            colors = np.asarray(cloud.colors)
            pointsWithColor = []
            for i in range(len(colors)):
                if colors[i, :].any():
                    pointsWithColor.append(i)
            cloud = cloud.select_by_index(pointsWithColor)

        if voxelSize > 0:
            cloud = cloud.voxel_down_sample(voxelSize)

        return cloud

    def updateWorldPose(self, camToWorld):
        prevWorldToCam = np.linalg.inv(self.camToWorld)
        prevToCurrent = np.matmul(camToWorld, prevWorldToCam)
        self.cloud.transform(prevToCurrent)
        self.camToWorld = camToWorld

# Wrapper around Open3D coordinate frame, which helps updating its world pose.
class CoordinateFrame:
    def __init__(self, scale=0.25):
        self.frame = o3d.geometry.TriangleMesh.create_coordinate_frame(scale)
        self.camToWorld = np.identity(4)

    def updateWorldPose(self, camToWorld):
        prevWorldToCam = np.linalg.inv(self.camToWorld)
        prevToCurrent = np.matmul(camToWorld, prevWorldToCam)
        self.frame.transform(prevToCurrent)
        self.camToWorld = camToWorld

class Open3DVisualization:
    def __init__(self, voxelSize, cameraManual, cameraSmooth, colorOnly):
        self.shouldClose = False
        self.pointClouds = {}
        self.cameraFrame = CoordinateFrame()
        self.vis = o3d.visualization.Visualizer()
        self.voxelSize = voxelSize
        self.cameraFollow = not cameraManual
        self.cameraSmooth = cameraSmooth
        self.colorOnly = colorOnly
        self.prevPos = None
        self.prevCamPos = None

    def run(self):
        self.vis.create_window()
        self.vis.add_geometry(self.cameraFrame.frame, reset_bounding_box=False)
        self.viewControl = self.vis.get_view_control()
        renderOption = self.vis.get_render_option()
        renderOption.point_size = 2
        renderOption.light_on = False

        print("Close the window to stop mapping")
        while not self.shouldClose:
            self.shouldClose = not self.vis.poll_events()
            self.vis.update_renderer()
            time.sleep(0.01)

        self.vis.destroy_window()

    def updateCameraFrame(self, camToWorld):
        self.cameraFrame.updateWorldPose(camToWorld)
        self.vis.update_geometry(self.cameraFrame.frame)

        if self.cameraFollow:
            pos = camToWorld[0:3, 3]
            forward = camToWorld[0:3, 2]
            upVector = np.array([0, 0, 1])
            camPos = pos - forward * 0.1 + upVector * 0.05

            if self.cameraSmooth and self.prevPos is not None:
                alpha = np.array([0.01, 0.01, 0.001])
                camPos = camPos * alpha + self.prevCamPos * (np.array([1, 1, 1])  - alpha)
                pos = pos * alpha + self.prevPos * (np.array([1, 1, 1]) - alpha)

            self.prevPos = pos
            self.prevCamPos = camPos

            viewDir = pos - camPos
            viewDir /= np.linalg.norm(viewDir)
            leftDir = np.cross(upVector, viewDir)
            upDir = np.cross(viewDir, leftDir)
            self.viewControl.set_lookat(pos)
            self.viewControl.set_front(-viewDir)
            self.viewControl.set_up(upDir)
            self.viewControl.set_zoom(0.3)

    def containsKeyFrame(self, keyFrameId):
        return keyFrameId in self.pointClouds

    def addKeyFrame(self, keyFrameId, keyFrame):
        camToWorld = keyFrame.frameSet.primaryFrame.cameraPose.getCameraToWorldMatrix()

        pc = PointCloud(keyFrame, self.voxelSize, self.colorOnly)
        pc.updateWorldPose(camToWorld)
        self.pointClouds[keyFrameId] = pc

        reset = len(self.pointClouds) == 1
        self.vis.add_geometry(pc.cloud, reset_bounding_box=reset)

    def updateKeyFrame(self, keyFrameId, keyFrame):
        camToWorld = keyFrame.frameSet.primaryFrame.cameraPose.getCameraToWorldMatrix()

        pc = self.pointClouds[keyFrameId]
        pc.updateWorldPose(camToWorld)

        self.vis.update_geometry(pc.cloud)

    def removeKeyFrame(self, keyFrameId):
        pc = self.pointClouds.pop(keyFrameId)
        self.vis.remove_geometry(pc.cloud, reset_bounding_box=False)

def parseArgs():
    import argparse
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("--dataFolder", help="Folder containing the recorded session for mapping")
    p.add_argument("--outputFolder", help="Folder where to save the captured point clouds")
    p.add_argument("--voxel", help="Voxel size (m) for downsampling point clouds")
    p.add_argument("--manual", help="Control Open3D camera manually", action="store_true")
    p.add_argument("--smooth", help="Apply some smoothing to 3rd person camera movement", action="store_true")
    p.add_argument("--color", help="Filter points without color", action="store_true")
    return p.parse_args()

if __name__ == '__main__':
    args = parseArgs()
    if args.outputFolder:
        os.makedirs(args.outputFolder)
    voxelSize = 0 if args.voxel is None else float(args.voxel)
    visu3D = Open3DVisualization(voxelSize, args.manual, args.smooth, args.color)

    def onVioOutput(vioOutput):
        cameraPose = vioOutput.getCameraPose(0)
        camToWorld = cameraPose.getCameraToWorldMatrix()
        visu3D.updateCameraFrame(camToWorld)

    def onMappingOutput(output):
        for frameId in output.updatedKeyFrames:
            keyFrame = output.map.keyFrames.get(frameId)

            # Remove deleted key frames from visualisation
            if not keyFrame:
                if visu3D.containsKeyFrame(frameId):
                    visu3D.removeKeyFrame(frameId)
                continue

            # Check that point cloud exists
            if not keyFrame.pointCloud:
                continue

            # Render key frame point clouds
            if visu3D.containsKeyFrame(frameId):
                # Existing key frame
                visu3D.updateKeyFrame(frameId, keyFrame)
            else:
                # New key frame
                visu3D.addKeyFrame(frameId, keyFrame)

        if output.finalMap:
            print("Final map ready!")

    def captureLoop():
        if args.dataFolder:
            print("Starting replay")
            replay = spectacularAI.Replay(args.dataFolder, onMappingOutput)
            replay.setOutputCallback(onVioOutput)
            replay.startReplay()
        else:
            print("Starting OAK-D device")
            pipeline = depthai.Pipeline()
            config = spectacularAI.depthai.Configuration()
            config.internalParameters = {
                "computeStereoPointCloud": "true",
                "pointCloudNormalsEnabled": "true"}
            vioPipeline = spectacularAI.depthai.Pipeline(pipeline, config, onMappingOutput)

            with depthai.Device(pipeline) as device, \
                vioPipeline.startSession(device) as vio_session:
                while not visu3D.shouldClose:
                    onVioOutput(vio_session.waitForOutput())

    thread = threading.Thread(target = captureLoop)
    thread.start()
    visu3D.run()
    thread.join()

    if args.outputFolder:
        print("Saving point clouds to {0}".format(args.outputFolder))
        for id in visu3D.pointClouds:
            pc = visu3D.pointClouds[id]
            filename = "{0}/{1}.ply".format(args.outputFolder, id)
            o3d.io.write_point_cloud(filename, pc.cloud)
