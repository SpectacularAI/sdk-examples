import depthai
import spectacularAI
import time

pipeline = depthai.Pipeline()

vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, useStereo=False)
# optional config args: vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, config, useStereo=False)

with depthai.Device(pipeline) as device:
    vio_session = vio_pipeline.startSession(device)

    while True:
        if vio_session.hasOutput():
            out = vio_session.getOutput()
            print(out.asJson())
        else:
            if not vio_session.work():
                time.sleep(0.005)
