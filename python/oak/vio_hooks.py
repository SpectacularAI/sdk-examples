"""
An example of accessing the raw data feeds from DepthAI that the SDK uses through
the Spectacualar AI hooks API. It's important to note that these are only called
if the particular mode the Spetacular AI SDK is running in uses that data feed
"""

import depthai
import spectacularAI
import cv2
import numpy as np

pipeline = depthai.Pipeline()

config = spectacularAI.depthai.Configuration()
config.useSlam = True

vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, config)

featureBuffer = None

def onImageFactor(name):
    def onImage(img):
        global featureBuffer
        if img.getWidth() <= 0 or img.getHeight() <= 0:
            # When SLAM is enabled, monocular frames are only used at 1/6th of normal frame rate,
            # rest of the frames are [0,0] in size and must be filtered
            return
        if type(featureBuffer) is not np.ndarray: featureBuffer = np.zeros((img.getHeight(), img.getWidth(), 1), dtype = "uint8")
        cv2.imshow(name, img.getCvFrame())
        if cv2.waitKey(1) == ord("q"):
            exit(0)
    return onImage

def onImuData(imuData):
    for imuPacket in imuData.packets:
        acceleroValues = imuPacket.acceleroMeter
        gyroValues = imuPacket.gyroscope
        acceleroTs = acceleroValues.getTimestampDevice().total_seconds() * 1000
        gyroTs = gyroValues.getTimestampDevice().total_seconds() * 1000
        imuF = "{:.06f}"
        tsF  = "{:.03f}"
        print(f"Accelerometer timestamp: {tsF.format(acceleroTs)} ms")
        print(f"Accelerometer [m/s^2]: x: {imuF.format(acceleroValues.x)} y: {imuF.format(acceleroValues.y)} z: {imuF.format(acceleroValues.z)}")
        print(f"Gyroscope timestamp: {tsF.format(gyroTs)} ms")
        print(f"Gyroscope [rad/s]: x: {imuF.format(gyroValues.x)} y: {imuF.format(gyroValues.y)} z: {imuF.format(gyroValues.z)} ")

def onFeatures(features):
    global featureBuffer
    if type(featureBuffer) is not np.ndarray: return
    featureBuffer[:] = 0
    for feature in features.trackedFeatures:
        cv2.circle(featureBuffer, (int(feature.position.x), int(feature.position.y)), 2, 255, -1, cv2.LINE_AA, 0)
    cv2.imshow("Features", featureBuffer)
    if cv2.waitKey(1) == ord("q"):
        exit(0)

vio_pipeline.hooks.imu = onImuData
vio_pipeline.hooks.monoPrimary = onImageFactor("Primary")
vio_pipeline.hooks.monoSecondary = onImageFactor("Secondary")
vio_pipeline.hooks.depth = onImageFactor("Depth")
vio_pipeline.hooks.trackedFeatures = onFeatures
# In default mode the color is not used by the SDK, so this will never get called.
# see mixed_reality.py example how to read the color data in an efficient manner.
vio_pipeline.hooks.color = onImageFactor("Color")

with depthai.Device(pipeline) as device, \
    vio_pipeline.startSession(device) as vio_session:

    while True:
        out = vio_session.waitForOutput()
        print(out.asJson())
