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
import time

config = spectacularAI.depthai.Configuration()

p = argparse.ArgumentParser(__doc__)
p.add_argument("--output", help="Recording output folder", default="data")
p.add_argument('--auto_subfolders', action='store_true',
    help='Create timestamp-named subfolders for each recording')
p.add_argument("--use_rgb", help="Use RGB data for tracking (OAK-D S2)", action="store_true")
p.add_argument("--mono", help="Use a single camera (not stereo)", action="store_true")
p.add_argument("--no_rgb", help="Disable recording RGB video feed", action="store_true")
p.add_argument("--no_inputs", help="Disable recording JSONL and depth", action="store_true")
p.add_argument("--gray", help="Record (rectified) gray video data", action="store_true")
p.add_argument("--no_convert", help="Skip converting h265 video file", action="store_true")
p.add_argument('--no_preview', help='Do not show a live preview', action="store_true")
p.add_argument('--no_slam', help='Record with SLAM module disabled', action="store_true")
p.add_argument('--recording_only', help='Do not run VIO, may be faster', action="store_true")
p.add_argument('--disable_cameras', help='Prevents SDK from using cameras, for example to only record RGB camera and IMU', action="store_true")
# This can reduce CPU load while recording with the --no_feature_tracker option
# and the 800p resolution. See "ffmpeg -codecs" (and see "encoders" under h264)
# for options that might be available. On Raspberry Pi or Jetson, try "h264_v4l2m2m",
# and on Linux machines with Nvidia GPUs, try "h264_nvenc".
p.add_argument('--ffmpeg_codec', help="FFMpeg codec for host", default=None)
p.add_argument('--map', help='Record SLAM map', action="store_true")
p.add_argument('--no_feature_tracker', help='Disable on-device feature tracking', action="store_true")
p.add_argument('--ir_dot_brightness', help='OAK-D Pro (W) IR laser projector brightness (mA), 0 - 1200', type=float, default=0)
p.add_argument("--resolution", help="Gray input resolution (gray)",
    default=config.inputResolution,
    choices=['400p', '800p'])
args =  p.parse_args()

pipeline = depthai.Pipeline()

config.useSlam = True
config.inputResolution = args.resolution
outputFolder = args.output
if args.auto_subfolders:
    import datetime
    autoFolderName = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
    outputFolder = os.path.join(outputFolder, autoFolderName)

if not args.no_inputs:
    config.recordingFolder = outputFolder
if args.map:
    try: os.makedirs(outputFolder) # SLAM only
    except: pass
    config.mapSavePath = os.path.join(outputFolder, 'slam_map._')
if args.no_slam:
    assert args.map == False
    config.useSlam = False
if args.no_feature_tracker:
    config.useFeatureTracker = False
if args.use_rgb:
    config.useColor = True
if args.mono:
    config.useStereo = False
if args.recording_only:
    config.recordingOnly = True
if args.disable_cameras:
    config.disableCameras = True
if args.ffmpeg_codec is not None:
    config.internalParameters = { 'ffmpegVideoCodec': args.ffmpeg_codec + ' -b:v 8M' }
    print(config.internalParameters)

# Enable recoding by setting recordingFolder option
vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, config)

# Optionally also record other video streams not used by the Spectacular AI SDK, these
# can be used for example to render AR content or for debugging.
rgb_as_video = not args.no_rgb and not args.use_rgb
if rgb_as_video:
    import numpy # Required by frame.getData(), otherwise it hangs indefinitely
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

should_quit = threading.Event()
def main_loop(plotter=None):
    frame_number = 1

    with depthai.Device(pipeline) as device, \
        vio_pipeline.startSession(device) as vio_session:

        if args.ir_dot_brightness > 0:
            device.setIrLaserDotProjectorBrightness(args.ir_dot_brightness)

        def open_gray_video(name):
            grayVideoFile = open(outputFolder + '/rectified_' + name + '.h264', 'wb')
            queue = device.getOutputQueue(name='h264-' + name, maxSize=10, blocking=False)
            return (queue, grayVideoFile)

        grayVideos = []
        if args.gray:
            grayVideos = [
                open_gray_video('left'),
                open_gray_video('right')
            ]

        if rgb_as_video:
            videoFile = open(outputFolder + "/rgb_video.h265", "wb")
            rgbQueue = device.getOutputQueue(name="h265-rgb", maxSize=30, blocking=False)

        print("Recording!")
        print("")
        if plotter is not None:
            print("Close the visualization window to stop recording")

        while not should_quit.is_set():
            progress = False
            if rgb_as_video:
                if rgbQueue.has():
                    frame = rgbQueue.get()
                    vio_session.addTrigger(frame.getTimestamp().total_seconds(), frame_number)
                    frame.getData().tofile(videoFile)
                    frame_number += 1
                    progress = True

            for (grayQueue, grayVideoFile) in grayVideos:
                if grayQueue.has():
                    grayQueue.get().getData().tofile(grayVideoFile)
                    progress = True

            if vio_session.hasOutput():
                out = vio_session.getOutput()
                progress = True
                if plotter is not None:
                    if not plotter(json.loads(out.asJson())): break

            if not progress:
                time.sleep(0.01)

    videoFileNames = []

    if rgb_as_video:
        videoFileNames.append(videoFile.name)
        videoFile.close()

    for (_, grayVideoFile) in grayVideos:
        videoFileNames.append(grayVideoFile.name)
        grayVideoFile.close()

    for fn in videoFileNames:
        if not args.no_convert:
            withoutExt = fn.rpartition('.')[0]
            ffmpegCommand = "ffmpeg -framerate 30 -y -i  \"{}\" -avoid_negative_ts make_zero -c copy \"{}.mp4\"".format(fn, withoutExt)

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
else:
    plt.show()
should_quit.set()

reader_thread.join()
