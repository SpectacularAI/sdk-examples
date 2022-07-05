import argparse
import spectacularAI
import cv2

p = argparse.ArgumentParser(__doc__)
p.add_argument("dataFolder", help="Folder containing the recorded session for replay", default="data")
args =  p.parse_args()

def onOutput(output, frameSet):
    for frame in frameSet:
        if frame.image:
            cv2.imshow("Camera #{}".format(frame.index), cv2.cvtColor(frame.image.toArray(), cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)
    print(output.asJson())

replay = spectacularAI.Replay(args.dataFolder)
# If frameSet isn't used, it's better to use setOutputCallback(...) which is lighter
# replay.setOutputCallback(onOutput)
replay.setExtendedOutputCallback(onOutput)
replay.runReplay()
