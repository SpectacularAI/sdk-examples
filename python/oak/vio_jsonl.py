import depthai
import spectacularAI
import time

pipeline = depthai.Pipeline()

vio_pipeline = spectacularAI.depthai.Pipeline(pipeline)
# optional config args: vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, config, useStereo=False)

with depthai.Device(pipeline) as device, \
    vio_pipeline.startSession(device) as vio_session:

    while True:
        out = vio_session.waitForOutput()
        print(out.asJson())
