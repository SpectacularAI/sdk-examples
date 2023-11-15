"""
An example code to deserialize data serialized by cpp/mapping_visu C++ example
"""

import struct
import json
import numpy as np
def input_stream_reader(in_stream):
    MAGIC_BYTES = 2727221974
    while True:
        messageHeader = in_stream.read(16)
        if len(messageHeader) < 16: break # EOF

        magicBytes, messageId, jsonSize, binarySize = struct.unpack('@4I', messageHeader)
        if magicBytes != MAGIC_BYTES:
            raise Exception(f"Wrong magic bytes! Expected {MAGIC_BYTES} and received {magicBytes}")
        json_output = json.loads(in_stream.read(jsonSize).decode('ascii'))

        if 'cameraPoses' in json_output: # Vio output
            in_stream.read(binarySize) # Dump any binary data
        else: # Mapper output, deserialize binary data
            for keyFrameId in json_output["updatedKeyFrames"]:
                keyFrame = json_output["map"]["keyFrames"].get(str(keyFrameId))
                if not keyFrame: continue # Deleted key frame
                if "pointCloud" in keyFrame:
                    pointCloud = keyFrame["pointCloud"]
                    points = pointCloud["size"]
                    pointCloud["positionData"] = np.frombuffer(in_stream.read(points * 4 * 3), dtype=np.float32)
                    pointCloud["positionData"].shape = (points, 3)
                    if pointCloud["hasNormals"]:
                        pointCloud["normalData"] = np.frombuffer(in_stream.read(points * 4 * 3), dtype=np.float32)
                        pointCloud["normalData"].shape = (points, 3)
                    if pointCloud["hasColors"]:
                        pointCloud["rgb24Data"] = np.frombuffer(in_stream.read(points * 3), dtype=np.ubyte)
                        pointCloud["rgb24Data"].shape = (points, 3)

        yield json_output

class MockCameraPose:
    def __init__(self, data):
        self.data = data
    def getCameraToWorldMatrix(self):
        return np.array(self.data["cameraToWorld"])

class MockVioOutput:
    def __init__(self, data):
        self.data = data
    def getCameraPose(self, index):
        return MockCameraPose(self.data["cameraPoses"][index])

class MockFrame:
    def __init__(self, data):
        self.cameraPose = MockCameraPose(data["cameraPose"])

class MockFrameSet:
    def __init__(self, data):
        self.primaryFrame = MockFrame(data["primaryFrame"])

class MockPointCloud:
    def __init__(self, data):
        self.data = data
    def getPositionData(self):
        return self.data["positionData"]
    def getRGB24Data(self):
        return self.data["rgb24Data"]
    def getNormalData(self):
        return self.data["normalData"]
    def hasColors(self):
        return 'rgb24Data' in self.data
    def hasNormals(self):
        return 'normalData' in self.data

class MockKeyFrame:
    def __init__(self, data):
        self.frameSet = MockFrameSet(data["frameSet"])
        if "pointCloud" in data:
            self.pointCloud = MockPointCloud(data["pointCloud"])
        else:
            self.pointCloud = None

class MockMap:
    def __init__(self, data):
        self.keyFrames = {}
        for keyFrameId in data["keyFrames"]:
            keyFrame = data["keyFrames"][keyFrameId]
            self.keyFrames[int(keyFrameId)] = MockKeyFrame(keyFrame)

class MockMapperOutput:
    def __init__(self, data):
        self.updatedKeyFrames = data["updatedKeyFrames"]
        self.finalMap = data["finalMap"]
        self.map = MockMap(data["map"])

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--file', type=argparse.FileType('rb'),
                        help='Read data from file or pipe',
                        default=None)
    args = parser.parse_args()
    vio_source = input_stream_reader(args.file)
    for vio_out in vio_source:
        # Do something with output
        if 'cameraPoses' in vio_out:
            vioOutput = MockVioOutput(vio_out)
            print(vioOutput.getCameraPose(0).getCameraToWorldMatrix())
        else:
            mapperOutput = MockMapperOutput(vio_out)
            print(f"Updated keyframes: {mapperOutput.updatedKeyFrames}")
