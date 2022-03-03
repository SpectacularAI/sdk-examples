"""
Record data for later playback

Requirements:

    ffmpeg must be installed.

    On Linux you can install it with package manager
    of your choise. For example with
        ap-get:   sudo apt-get install ffmpeg
        yuM:      sudo yum install ffmpeg

On Windows, you must download and install it from https://www.ffmpeg.org and
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
import json
import threading

config = spectacularAI.depthai.Configuration()

p = argparse.ArgumentParser(__doc__)
p.add_argument("--output", help="Recording output folder", default="data")
p.add_argument("--no_rgb", help="Disable recording RGB video feed", action="store_true")
p.add_argument("--no_inputs", help="Disable recording JSONL and depth", action="store_true")
p.add_argument("--gray", help="Record (rectified) gray video data", action="store_true")
p.add_argument("--no_convert", help="Skip converting h265 video file", action="store_true")
p.add_argument('--no_preview', help='Do not show a live preview', action="store_true")
p.add_argument('--slam', help='Record SLAM map', action="store_true")
p.add_argument("--resolution", help="Gray input resolution (gray)",
    default=config.inputResolution,
    choices=['400p', '800p'])
args =  p.parse_args()

pipeline = depthai.Pipeline()

config.inputResolution = args.resolution
if not args.no_inputs:
    config.recordingFolder = args.output
if args.slam:
    config.useSlam = True
    try: os.makedirs(args.output) # SLAM only
    except: pass
    config.mapSavePath = os.path.join(args.output, 'slam_map._')

# Enable recoding by setting recordingFolder option
vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, config)

# Optionally also record other video streams not used by the Spectacular AI SDK, these
# can be used for example to render AR content or for debugging.
if not args.no_rgb:
    camRgb = pipeline.create(depthai.node.ColorCamera)
    videoEnc = pipeline.create(depthai.node.VideoEncoder)
    xout = pipeline.create(depthai.node.XLinkOut)
    xout.setStreamName("h265-rgb")
    camRgb.setBoardSocket(depthai.CameraBoardSocket.RGB)
    camRgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
    # no need to set input resolution anymore (update your depthai package if this does not work)
    videoEnc.setDefaultProfilePreset(30, depthai.VideoEncoderProperties.Profile.H265_MAIN)
    camRgb.video.link(videoEnc.input)
    videoEnc.bitstream.link(xout.input)

if args.gray:
    def create_gray_encoder(node, name):
        videoEnc = pipeline.create(depthai.node.VideoEncoder)
        xout = pipeline.create(depthai.node.XLinkOut)
        xout.setStreamName("h264-" + name)
        videoEnc.setDefaultProfilePreset(30, depthai.VideoEncoderProperties.Profile.H264_MAIN)
        node.link(videoEnc.input)
        videoEnc.bitstream.link(xout.input)

    create_gray_encoder(vio_pipeline.stereo.rectifiedLeft, 'left')
    create_gray_encoder(vio_pipeline.stereo.rectifiedRight, 'right')

should_quit = False
def main_loop(plotter=None):
    frame_number = 1

    with depthai.Device(pipeline) as device, \
        vio_pipeline.startSession(device) as vio_session:

        def open_gray_video(name):
            grayVideoFile = open(args.output + '/rectified_' + name + '.h264', 'wb')
            queue = device.getOutputQueue(name='h264-' + name, maxSize=10, blocking=False)
            return (queue, grayVideoFile)

        grayVideos = []
        if args.gray:
            grayVideos = [
                open_gray_video('left'),
                open_gray_video('right')
            ]

        if not args.no_rgb:
            videoFile = open(args.output + "/rgb_video.h265", "wb")
            rgbQueue = device.getOutputQueue(name="h265-rgb", maxSize=30, blocking=False)

        print("Recording!")
        print("")
        if plotter is not None:
            print("Close the visualization window to stop recording")

        while not should_quit:
            if not args.no_rgb:
                while rgbQueue.has():
                    frame = rgbQueue.get()
                    vio_session.addTrigger(frame.getTimestamp().total_seconds(), frame_number)
                    frame.getData().tofile(videoFile)
                    frame_number += 1

            for (grayQueue, grayVideoFile) in grayVideos:
                if grayQueue.has():
                    grayQueue.get().getData().tofile(grayVideoFile)

            out = vio_session.waitForOutput()
            if plotter is not None:
                if not plotter(json.loads(out.asJson())): break

    videoFileNames = []

    if not args.no_rgb:
        videoFileNames.append(videoFile.name)
        videoFile.close()

    for (_, grayVideoFile) in grayVideos:
        videoFileNames.append(grayVideoFile.name)
        grayVideoFile.close()

    for fn in videoFileNames:
        if not args.no_convert:
            withoutExt = fn.rpartition('.')[0]
            ffmpegCommand = "ffmpeg -framerate 30 -y -i {} -avoid_negative_ts make_zero -c copy {}.mp4".format(fn, withoutExt)

            result = subprocess.run(ffmpegCommand, shell=True)
            if result.returncode == 0:
                os.remove(fn)
        else:
            print('')
            print("Use ffmpeg to convert video into a viewable format:")
            print("    " + ffmpegCommand)

if args.no_preview:
    plotter = None
else:
    from vio_visu import make_plotter
    import matplotlib.pyplot as plt
    plotter, anim = make_plotter()

reader_thread = threading.Thread(target = lambda: main_loop(plotter))
reader_thread.start()
if plotter is None:
    input("---- Press ENTER to stop recording ----")
    should_quit = True
else:
    plt.show()

reader_thread.join()
