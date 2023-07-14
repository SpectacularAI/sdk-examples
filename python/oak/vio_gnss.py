"""
    GNSS-VIO fusion example. Receives GNSS positions through STDIN in JSON format
    and sends them to Vio.

    An example GNSS message. Either "accuracy" or "enuPositionCovariance" must be
    provided.

    {
        "latitude": 60.0,
        "longitude": 24.0,
        "altitude": 2.0,
        "monotonicTime": 123456.789
        "accuracy": 0.10,
        "verticalAccuracy": 0.20,
        "enuPositionCovariance": [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 5],
        ]
    }
"""

import depthai
import spectacularAI
import sys
import threading
import json
import sys

# A separate thread that reads gnss position as JSON from stdin and
# sends it to vio_session
def gnssInput(vio_session):
    def computeSystemToDeviceTimeOffset(device):
        imu_queue = device.getOutputQueue(name="spectacularAI_imu", maxSize=1, blocking=True)
        imu_data = imu_queue.get()
        acc = imu_data.packets[0].acceleroMeter
        ts_device = acc.getTimestampDevice().total_seconds()
        ts_system = acc.getTimestamp().total_seconds()
        return ts_device - ts_system

    system_to_device_time_offset = computeSystemToDeviceTimeOffset(device)

    for line in sys.stdin:
        gnss = json.loads(line)
        coordinates = spectacularAI.WgsCoordinates()
        coordinates.latitude = gnss["latitude"]
        coordinates.longitude = gnss["longitude"]
        coordinates.altitude = gnss["altitude"]
        enuPositionCovariance =  gnss.get("enuPositionCovariance")
        if not enuPositionCovariance:
            horizontalStddev = gnss["accuracy"]
            verticalStddev = gnss.get("verticalAccuracy")
            if not verticalStddev: verticalStddev = horizontalStddev
            enuPositionCovariance = [
                [horizontalStddev * horizontalStddev, 0, 0],
                [0, horizontalStddev * horizontalStddev, 0],
                [0, 0, verticalStddev * verticalStddev]
            ]

        # device_time = system_time + system_to_device_time_offset
        vio_session.addGnss(
            gnss["monotonicTime"] + system_to_device_time_offset,
            coordinates,
            enuPositionCovariance
        )

pipeline = depthai.Pipeline()
config = spectacularAI.depthai.Configuration()

if len(sys.argv) >= 2: config.recordingFolder = sys.argv[1]

# Optional imuToGnss translation in meters
#config.imuToGnss = spectacularAI.Vector3d(0.0, 0.0, 0.0)

vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, config)

with depthai.Device(pipeline) as device, \
    vio_pipeline.startSession(device) as vio_session:

    threading.Thread(target = gnssInput, args=(vio_session,)).start()

    while True:
        out = vio_session.waitForOutput()
        if out.globalPose:
            coords = out.globalPose.coordinates
            print("Global position: " + str(coords.latitude) + ", " + str(coords.longitude))
        else:
            print("No global position yet, local pose: " + out.asJson())
