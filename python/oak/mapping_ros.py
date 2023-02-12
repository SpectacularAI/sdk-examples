"""
Runs spectacularAI mapping and publishes poses and frames in ROS.
"""
import spectacularAI
import depthai
import rospy
import numpy as np
import threading
import time
import os
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped
from scipy.spatial.transform import Rotation
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def to_pose_message(camToWorld):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    msg.pose.position.x = camToWorld[0, 3]
    msg.pose.position.y = camToWorld[1, 3]
    msg.pose.position.z = camToWorld[2, 3]
    R_CW = Rotation.from_matrix(camToWorld[0:3, 0:3])
    q_cw = R_CW.as_quat()
    msg.pose.orientation.x = q_cw[0]
    msg.pose.orientation.y = q_cw[1]
    msg.pose.orientation.z = q_cw[2]
    msg.pose.orientation.w = q_cw[3]
    return msg

def to_tf_message(camToWorld, ts, frame_id):
    msg = TFMessage()
    msg.transforms = []
    transform = TransformStamped()
    transform.header.stamp = ts
    transform.header.frame_id = "world"
    transform.child_frame_id = frame_id
    transform.transform.translation.x = camToWorld[0, 3]
    transform.transform.translation.y = camToWorld[1, 3]
    transform.transform.translation.z = camToWorld[2, 3]
    R_CW = Rotation.from_matrix(camToWorld[0:3, 0:3])
    q_cw = R_CW.as_quat()
    transform.transform.rotation.x = q_cw[0]
    transform.transform.rotation.y = q_cw[1]
    transform.transform.rotation.z = q_cw[2]
    transform.transform.rotation.w = q_cw[3]
    msg.transforms.append(transform)
    return msg


class SLAMNode:
    def __init__(self):
        rospy.init_node("slam_node", anonymous=True)
        self.odometry_publisher = rospy.Publisher("/slam/odometry", PoseStamped, queue_size=10)
        self.keyframe_publisher = rospy.Publisher("/slam/keyframe", PoseStamped, queue_size=10)
        self.rgb_publisher = rospy.Publisher("/slam/rgb", Image, queue_size=10)
        self.tf_publisher = rospy.Publisher("/tf", TFMessage, queue_size=10)
        self.depth_publisher = rospy.Publisher("/slam/depth", Image, queue_size=10)
        self.bridge = CvBridge()
        self.keyframes = {}

    def has_keyframe(self, frame_id):
        return frame_id in self.keyframes

    def newKeyFrame(self, frame_id, keyframe):
        self.keyframes[frame_id] = True
        camToWorld = keyframe.frameSet.rgbFrame.cameraPose.getCameraToWorldMatrix()
        msg = to_pose_message(camToWorld)
        self.keyframe_publisher.publish(msg)
        rgb_frame = keyframe.frameSet.rgbFrame.image
        now = rospy.Time.now()
        if rgb_frame is not None:
            rgb_frame = rgb_frame.toArray()
            rgb_message = self.bridge.cv2_to_imgmsg(rgb_frame, encoding="rgb8")
            rgb_message.header.stamp = rospy.Time.now()
            rgb_message.header.frame_id = "rgb_optical"
            rgb_message.header.seq = int(frame_id)
            self.rgb_publisher.publish(rgb_message)
            tf_message = to_tf_message(camToWorld, now, "rgb_optical")
            self.tf_publisher.publish(tf_message)

        depth_frame = keyframe.frameSet.getAlignedDepthFrame(keyframe.frameSet.rgbFrame)
        depth = depth_frame.image.toArray()
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="mono16")
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = "rgb_optical"
        depth_msg.header.seq = int(frame_id)
        self.depth_publisher.publish(depth_msg)

    def updateKeyFrame(self, frame_id, keyframe):
        print(f"Updated key frame {frame_id}")
        #TODO publish updates
        pass

    def newOdometryFrame(self, camToWorld):
        msg = to_pose_message(camToWorld)
        self.odometry_publisher.publish(msg)


def parseArgs():
    import argparse
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("--smooth", help="Apply some smoothing to 3rd person camera movement", action="store_true")
    p.add_argument('--ir_dot_brightness', help='OAK-D Pro (W) IR laser projector brightness (mA), 0 - 1200', type=float, default=0)
    p.add_argument("--useRectification", help="--dataFolder option can also be used with some non-OAK-D recordings, but this parameter must be set if the videos inputs are not rectified.", action="store_true")
    return p.parse_args()

if __name__ == '__main__':
    args = parseArgs()

    configInternal = {
        "computeStereoPointCloud": "true",
        "pointCloudNormalsEnabled": "true",
        "computeDenseStereoDepth": "true",
    }
    if args.useRectification:
        configInternal["useRectification"] = "true"
    else:
        configInternal["alreadyRectified"] = "true"

    slam_node = SLAMNode()

    def onVioOutput(vioOutput):
        cameraPose = vioOutput.getCameraPose(0)
        camToWorld = cameraPose.getCameraToWorldMatrix()
        slam_node.newOdometryFrame(camToWorld)

    def onMappingOutput(output):
        for frame_id in output.updatedKeyFrames:
            keyFrame = output.map.keyFrames.get(frame_id)

            # Remove deleted key frames from visualisation
            if not keyFrame:
                continue

            # Check that point cloud exists
            if not keyFrame.pointCloud: continue

            if slam_node.has_keyframe(frame_id):
                slam_node.updateKeyFrame(frame_id, keyFrame)
            else:
                slam_node.newKeyFrame(frame_id, keyFrame)

        if output.finalMap:
            print("Final map ready!")

    print("Starting OAK-D device")
    pipeline = depthai.Pipeline()
    config = spectacularAI.depthai.Configuration()
    config.useColor = True
    config.internalParameters = configInternal
    config.useSlam = True
    config.useFeatureTracker = True
    config.keyframeCandidateEveryNthFrame = 6
    vioPipeline = spectacularAI.depthai.Pipeline(pipeline, config, onMappingOutput)

    with depthai.Device(pipeline) as device, \
        vioPipeline.startSession(device) as vio_session:
        if args.ir_dot_brightness > 0:
            device.setIrLaserDotProjectorBrightness(args.ir_dot_brightness)
        while not rospy.is_shutdown():
            onVioOutput(vio_session.waitForOutput())


