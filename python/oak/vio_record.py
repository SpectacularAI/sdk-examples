"""
Recording session for later playback

Requirements:

    ffmpeg must be installed.

    On Linux you can install it with package manager
    of your choise. For example with
        ap-get:   sudo apt-get install ffmpeg
        yuM:      sudo yum install ffmpeg

    On Windows you must download and install it from https://www.ffmpeg.org and
    then update your environment Path variable to contain the binary path. To do
    this, press Windows Key, type Path and press Enter. Open Environment Settings,
    edit the row named Path and add location of the ffmpeg bin folder to the list,
    for example: "C:\Program Files\ffmpeg\bin". To check that it works, open
    command prompt and type ffmpeg, you should see version information.

    To view the depth video file, you must use ffplay, because normal video players
    cannot play 16bit grayscale video.

Plug in the OAK-D and run:

    python examples/vio_record.py

"""

import depthai
import spectacularAI
import signal
import sys
import argparse
import subprocess
import os

p = argparse.ArgumentParser(
    description="Record session")
p.add_argument("--output", help="Recording output folder", default="data")
p.add_argument("--norgb", help="Disable recording RGB video feed", action="store_true")
p.add_argument("--noconvert", help="Skip converting h265 video file", action="store_true")
args =  p.parse_args()

pipeline = depthai.Pipeline()

# Enable recoding by setting recordingFolder option
vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, recordingFolder=args.output)

# Optionally also record other video streams not used by the Spectacular AI SDK, these
# can be used for example to render AR content or for debugging.
if not args.norgb:
    camRgb = pipeline.create(depthai.node.ColorCamera)
    videoEnc = pipeline.create(depthai.node.VideoEncoder)
    xout = pipeline.create(depthai.node.XLinkOut)
    xout.setStreamName("h265")
    camRgb.setBoardSocket(depthai.CameraBoardSocket.RGB)
    camRgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
    videoEnc.setDefaultProfilePreset(1920, 1080, 30, depthai.VideoEncoderProperties.Profile.H265_MAIN)
    camRgb.video.link(videoEnc.input)
    videoEnc.bitstream.link(xout.input)

frame_number = 1

try:
    with depthai.Device(pipeline) as device, \
        vio_pipeline.startSession(device) as vio_session, \
        open(args.output + "/rgb_video.h265", "wb") as videoFile:

        if not args.norgb: outQ = device.getOutputQueue(name="h265", maxSize=30, blocking=False)

        print("Recording!")
        print("")
        print("Press Ctrl+C to stop recording")

        while True:
            if not args.norgb:
                while outQ.has():
                    frame = outQ.get()
                    vio_session.addTrigger(frame.getTimestamp().total_seconds(), frame_number)
                    frame.getData().tofile(videoFile)
                    frame_number += 1
            out = vio_session.waitForOutput()

except KeyboardInterrupt:
    ffmpegCommand = "ffmpeg -framerate 30 -i {} -c copy {}".format(args.output + "/rgb_video.h265",
        args.output + "/rgb_video.mp4")
    if not args.noconvert:
        result = subprocess.run(ffmpegCommand, shell=True)
        if result.returncode == 0: os.remove(args.output + "/rgb_video.h265")
    else:
        print("")
        print("Use ffmpeg to convert video into a viewable format:")
        print("    " + ffmpegCommand)

