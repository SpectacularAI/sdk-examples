"""
Visualize 3D point cloud of the environment in real-time, or playback your recordings and view their 3D point cloud.

Requirements: pygame, numpy, PyOpenGL
Optional: PyOpenGL_accelerate

"""

import spectacularAI
import depthai
import threading

from spectacularAI.cli.visualization.serialization import input_stream_reader, MockVioOutput, MockMapperOutput
from spectacularAI.cli.visualization.visualizer import Visualizer, VisualizerArgs

def parseArgs():
    import argparse
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("--dataFolder", help="Instead of running live mapping session, replay session from this folder")
    p.add_argument('--file', type=argparse.FileType('rb'), help='Read data from file or pipe, using this with mapping_visu C++ example', default=None)
    p.add_argument("--recordingFolder", help="Record live mapping session for replay")
    p.add_argument("--resolution", help="Window resolution", default="1280x720")
    p.add_argument("--fullScreen", help="Start in full screen mode", action="store_true")
    p.add_argument("--recordWindow", help="Window recording filename")
    p.add_argument("--voxel", type=float, help="Voxel size (m) for downsampling point clouds")
    p.add_argument("--color", help="Filter points without color", action="store_true")
    p.add_argument("--useRgb", help="Use OAK-D RGB camera", action="store_true")
    p.add_argument('--irDotBrightness', help='OAK-D Pro (W) IR laser projector brightness (mA), 0 - 1200', type=float, default=0)
    p.add_argument('--noFeatureTracker', help='Disable on-device feature tracking and depth map', action="store_true")
    p.add_argument("--useRectification", help="--dataFolder option can also be used with some non-OAK-D recordings, but this parameter must be set if the videos inputs are not rectified.", action="store_true")
    p.add_argument('--keyFrameCandidateInterval', type=int, help='Sets internal parameter keyframeCandidateEveryNthFrame')
    return p.parse_args()

if __name__ == '__main__':
    args = parseArgs()

    configInternal = {
        "computeStereoPointCloud": "true",
        "pointCloudNormalsEnabled": "true",
        "computeDenseStereoDepth": "true",
    }
    if args.dataFolder and args.useRectification:
        configInternal["useRectification"] = "true"
    else:
        configInternal["alreadyRectified"] = "true"

    visArgs = VisualizerArgs()
    visArgs.resolution = args.resolution
    visArgs.fullScreen = args.fullScreen
    visArgs.recordPath = args.recordWindow
    visArgs.pointCloudVoxelSize = args.voxel
    visArgs.skipPointsWithoutColor = args.color
    visualizer = Visualizer(visArgs)

    def onMappingOutput(mapperOutput):
        visualizer.onMappingOutput(mapperOutput)
        if mapperOutput.finalMap: print("Final map ready!")

    def onVioOutput(vioOutput):
        visualizer.onVioOutput(vioOutput.getCameraPose(0))

    if args.file:
        print("Starting reading input from file or pipe")
        def inputStreamLoop():
            vio_source = input_stream_reader(args.file)
            for vio_out in vio_source:
                if 'cameraPoses' in vio_out: onVioOutput(MockVioOutput(vio_out))
                else: onMappingOutput(MockMapperOutput(vio_out))
        thread = threading.Thread(target=inputStreamLoop)
        thread.start()
        visualizer.run()
        thread.join()
    elif args.dataFolder:
        print("Starting replay")
        replay = spectacularAI.Replay(args.dataFolder, onMappingOutput, configuration=configInternal)
        replay.setOutputCallback(onVioOutput)
        replay.startReplay()
        visualizer.run()
        replay.close()
    else:
        def captureLoop():
            print("Starting OAK-D device")
            pipeline = depthai.Pipeline()
            config = spectacularAI.depthai.Configuration()
            config.useFeatureTracker = not args.noFeatureTracker
            config.useColor = args.useRgb
            config.internalParameters = configInternal
            if args.recordingFolder: config.recordingFolder = args.recordingFolder
            if args.keyFrameCandidateInterval: config.keyframeCandidateEveryNthFrame = args.keyFrameCandidateInterval
            vioPipeline = spectacularAI.depthai.Pipeline(pipeline, config, onMappingOutput)

            with depthai.Device(pipeline) as device, \
                vioPipeline.startSession(device) as vio_session:
                if args.irDotBrightness > 0:
                    device.setIrLaserDotProjectorBrightness(args.irDotBrightness)
                while not visualizer.shouldQuit:
                    onVioOutput(vio_session.waitForOutput())

        thread = threading.Thread(target=captureLoop)
        thread.start()
        visualizer.run()
        thread.join()
