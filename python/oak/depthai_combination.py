"""
Spatial AI demo combining Spectacular AI VIO with Tiny YOLO object detection
accelerated on the OAK-D.

Requirements:

    pip install opencv-python matplotlib

To download the pre-trained NN model run following shell script (Git Bash recommended on Windows to run it):

    ./depthai_combination_install.sh

Plug in the OAK-D and run:

    python examples/depthai_combination.py

"""
import depthai as dai
import time
import cv2
import matplotlib.pyplot as plt
import spectacularAI
import threading
from pathlib import Path
import sys
import numpy as np

def make_pipelines(nnBlobPath, showRgb):
    syncNN = True

    # Create pipeline
    pipeline = dai.Pipeline()
    vio_pipeline = spectacularAI.depthai.Pipeline(pipeline)

    # Define sources and outputs
    camRgb = pipeline.createColorCamera()
    spatialDetectionNetwork = pipeline.createYoloSpatialDetectionNetwork()

    if showRgb:
        xoutRgb = pipeline.createXLinkOut()
    xoutNN = pipeline.createXLinkOut()
    xoutBoundingBoxDepthMapping = pipeline.createXLinkOut()

    if showRgb:
        xoutRgb.setStreamName("rgb")
    xoutNN.setStreamName("detections")
    xoutBoundingBoxDepthMapping.setStreamName("boundingBoxDepthMapping")

    # Properties
    camRgb.setPreviewSize(416, 416)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    spatialDetectionNetwork.setBlobPath(nnBlobPath)
    spatialDetectionNetwork.setConfidenceThreshold(0.5)
    spatialDetectionNetwork.input.setBlocking(False)
    spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
    spatialDetectionNetwork.setDepthLowerThreshold(100)
    spatialDetectionNetwork.setDepthUpperThreshold(5000)

    # Yolo specific parameters
    spatialDetectionNetwork.setNumClasses(80)
    spatialDetectionNetwork.setCoordinateSize(4)
    spatialDetectionNetwork.setAnchors(np.array([10,14, 23,27, 37,58, 81,82, 135,169, 344,319]))
    spatialDetectionNetwork.setAnchorMasks({ "side26": np.array([1,2,3]), "side13": np.array([3,4,5]) })
    spatialDetectionNetwork.setIouThreshold(0.5)

    camRgb.preview.link(spatialDetectionNetwork.input)
    if showRgb:
        if syncNN:
            spatialDetectionNetwork.passthrough.link(xoutRgb.input)
        else:
            camRgb.preview.link(xoutRgb.input)

    spatialDetectionNetwork.out.link(xoutNN.input)
    spatialDetectionNetwork.boundingBoxMapping.link(xoutBoundingBoxDepthMapping.input)

    vio_pipeline.stereo.depth.link(spatialDetectionNetwork.inputDepth)

    return pipeline, vio_pipeline

def make_tracker():
    """
    Simple tracker/smoother/clustring for the YOLO-detected objects.
    (The raw YOLO results look quite, well, raw, especially in 3D)
    """
    tracked_objects = []
    next_id = 1

    class TrackedObject:
        def __init__(self, t, p, l):
            self.position = p
            self.label = l
            self.last_seen = t
            self.n_detections = 1

            nonlocal next_id
            self.id = next_id
            next_id += 1

        def update(self, other):
            UPDATE_ALPHA = 0.2
            self.last_seen = other.last_seen
            self.position = UPDATE_ALPHA * other.position + (1.0 - UPDATE_ALPHA) * self.position
            self.n_detections += 1

        def __repr__(self):
            return '%s %d' % (self.label, self.id)

    CLUSTERING_DISTANCE_AT_1M = 0.3

    def find_best_match(new_obj, w_to_c_mat):
        best = None
        best_dist = CLUSTERING_DISTANCE_AT_1M
        MIN_DEPTH = 0.5

        local_pos = lambda p: (w_to_c_mat @ np.array(list(p) + [1]))[:3]

        for old in tracked_objects:
            if old.label != new_obj.label: continue

            # ignore depth difference in clustering
            loc_old = local_pos(old.position)
            loc_new = local_pos(new_obj.position)
            z = max([MIN_DEPTH, loc_old[2], loc_new[2]])
            dist = np.linalg.norm((loc_old - loc_new)[:2]) / z

            if dist < best_dist:
                best_dist = dist
                best = old
        # if best: print(f'matched with {best} (seen {best.n_detections} time(s))')
        return best

    def track(t, detections, view_mat):
        SCALE = 0.001 # output is millimeters
        MIN_DETECTIONS = 8
        DETECTION_WINDOW = 1.0
        MAX_UNSEEN_AGE = 8.0

        w_to_c_mat = np.linalg.inv(view_mat)

        for d in detections:
            p_local = np.array([
                d.spatialCoordinates.x * SCALE,
                -d.spatialCoordinates.y * SCALE, # note: flipped y
                d.spatialCoordinates.z * SCALE,
                1
            ])
            p_world = (view_mat @ p_local)[:3]
            try:
                label = LABEL_MAP[d.label]
            except:
                label = d.label

            # simple O(n^2)
            for o in tracked_objects:
                if o.label != label: continue
                dist = np.linalg.norm(o.position - p_world)

            if label in SELECTED_LABELS:
                new_obj = TrackedObject(t, p_world, label)
                existing = find_best_match(new_obj, w_to_c_mat)
                if existing:
                    existing.update(new_obj)
                else:
                    tracked_objects.append(new_obj)

        def should_remove(o):
            if o.n_detections < MIN_DETECTIONS and o.last_seen < t - DETECTION_WINDOW: return True
            if o.last_seen < t - MAX_UNSEEN_AGE: return True
            return False

        # remove cruft
        i = 0
        while i < len(tracked_objects):
            if should_remove(tracked_objects[i]):
                # print(f'removing ${o}')
                del tracked_objects[i]
            else:
                i += 1

        # print(tracked_objects)
        return [o for o in tracked_objects if o.n_detections >= MIN_DETECTIONS]

    return track

# Tiny yolo v3/4 label texts
LABEL_MAP = [
    "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
    "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
    "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
    "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
    "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
    "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
    "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
    "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
    "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
    "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
    "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
    "teddy bear",     "hair drier", "toothbrush"
]

SELECTED_LABELS = ['mouse', 'cup', 'dog']

def make_camera_wireframe(aspect=640/400., scale=0.05):
    # camera "frustum"
    corners = [[-1, -1], [1, -1], [1, 1], [-1, 1], [-1, -1]]
    cam_wire = []
    for x, y in corners:
        cam_wire.append([x*aspect, y, 1])
    for x, y in corners:
        cam_wire.append([x*aspect, y, 1])
        cam_wire.append([0, 0, 0])
    return (scale * np.array(cam_wire)).tolist()

class MatplotlibVisualization:
    """
    Interactive / real-time 3D line & point visualization using Matplotlib.
    This is quite far from the comfort zone of MPL and not very extensible.
    """
    def __init__(self):
        from mpl_toolkits.mplot3d import Axes3D
        from matplotlib.animation import FuncAnimation

        fig = plt.figure()
        ax = Axes3D(fig, auto_add_to_figure=False)
        fig.add_axes(ax)

        ax_bounds = (-0.5, 0.5) # meters
        ax.set(xlim=ax_bounds, ylim=ax_bounds, zlim=ax_bounds)
        ax.view_init(azim=-140) # initial plot orientation

        empty_xyz = lambda: { c: [] for c in 'xyz' }

        vio_data = empty_xyz()
        vio_data['plot'] = ax.plot(
            xs=[], ys=[], zs=[],
            linestyle="-",
            marker="",
            label='VIO trajectory'
        )

        vio_cam_data = empty_xyz()
        vio_cam_data['plot'] = ax.plot(
            xs=[], ys=[], zs=[],
            linestyle="-",
            marker="",
            label='current cam pose'
        )

        detection_data = empty_xyz()
        detection_data['labels'] = []
        detection_data['plot'] = ax.plot(
            xs=[], ys=[], zs=[],
            linestyle="",
            marker="o",
            label=' or '.join(SELECTED_LABELS)
        )

        ax.legend()
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_zlabel("z (m)")

        #title = ax.set_title("Spatial AI demo")
        def on_close(*args):
            self.should_close = True

        fig.canvas.mpl_connect('close_event', on_close)

        self.cam_wire = make_camera_wireframe()
        self.vio_data = vio_data
        self.vio_cam_data = vio_cam_data
        self.detection_data = detection_data
        self.should_close = False

        def update_graph(*args):
            r = []
            for graph in [self.vio_data, self.vio_cam_data, self.detection_data]:
                p = graph['plot'][0]
                x, y, z = [np.array(graph[c]) for c in 'xyz']
                p.set_data(x, y)
                p.set_3d_properties(z)
                r.append(p)
            return tuple(r)

        self._anim = FuncAnimation(fig, update_graph, interval=15, blit=True)

    def update_vio(self, vio_out):
        if self.should_close: return False
        view_mat = vio_out.pose.asMatrix()

        for c in 'xyz': self.vio_cam_data[c] = []
        for vertex in self.cam_wire:
            p_local = np.array(vertex + [1])
            p_world = (view_mat @ p_local)[:3]
            for i, c in enumerate('xyz'):
                self.vio_cam_data[c].append(p_world[i])

        for c in 'xyz':
            self.vio_data[c].append(getattr(vio_out.pose.position, c))

        return True

    def update_detected_objects(self, tracked_objects):
        if self.should_close: return False

        for i in range(3):
            self.detection_data['xyz'[i]] = np.array([o.position[i] for o in tracked_objects])
        self.detection_data['labels'] = [o.label for o in tracked_objects]

        return True

    def start_in_parallel_with(self, parallel_thing):
        thread = threading.Thread(target = parallel_thing)
        thread.start()
        plt.show()
        thread.join()

def draw_detections_on_rgb_frame(frame, detections, fps):
    # If the frame is available, draw bounding boxes on it and show the frame
    height = frame.shape[0]
    width  = frame.shape[1]
    for detection in detections:
        # Denormalize bounding box
        x1 = int(detection.xmin * width)
        x2 = int(detection.xmax * width)
        y1 = int(detection.ymin * height)
        y2 = int(detection.ymax * height)
        try:
            label = LABEL_MAP[detection.label]
        except:
            label = detection.label
        if label in SELECTED_LABELS:
            color = (0, 255, 0)
            cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        else:
            color = (255, 0, 0)

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

    color = (255, 255, 255)
    cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)

if __name__ == '__main__':
    nnBlobPath = 'models/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob'
    if len(sys.argv) > 1:
        nnBlobPath = sys.argv[1]

    if not Path(nnBlobPath).exists():
        raise FileNotFoundError(f'Could not find {nnBlobPath}"')

    showRgb = True
    pipeline, vio_pipeline = make_pipelines(nnBlobPath, showRgb)

    with dai.Device(pipeline) as device:
        visu_3d = MatplotlibVisualization()

        def main_loop():
            startTime = time.monotonic()
            counter = 0
            fps = 0
            color = (255, 255, 255)

            vio_session = vio_pipeline.startSession(device)
            tracker = make_tracker()

            if showRgb: previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
            xoutBoundingBoxDepthMappingQueue = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False)

            vio_matrix = None

            while True:
                if vio_session.hasOutput():
                    vio_out = vio_session.getOutput()
                    vio_matrix = vio_out.pose.asMatrix()
                    if not visu_3d.update_vio(vio_out): break
                elif detectionNNQueue.has():
                    if showRgb:
                        inPreview = previewQueue.get()
                        frame = inPreview.getCvFrame()

                    inDet = detectionNNQueue.get()

                    # TODO: depth hook
                    #depthFrame = depth.getFrame()
                    #depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
                    #depthFrameColor = cv2.equalizeHist(depthFrameColor)
                    #depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

                    counter+=1
                    current_time = time.monotonic()
                    if (current_time - startTime) > 1 :
                        fps = counter / (current_time - startTime)
                        counter = 0
                        startTime = current_time

                    detections = inDet.detections
                    if len(detections) != 0:
                        boundingBoxMapping = xoutBoundingBoxDepthMappingQueue.get()
                        roiDatas = boundingBoxMapping.getConfigData()

                    if vio_matrix is not None:
                        detections_world = tracker(current_time, detections, vio_matrix)
                        visu_3d.update_detected_objects(detections_world)

                    if showRgb:
                        draw_detections_on_rgb_frame(frame, detections, fps)
                        cv2.imshow("rgb", frame)

                    if cv2.waitKey(1) == ord('q'):
                        break
                else:
                    time.sleep(0.005)

            vio_session.close()

        visu_3d.start_in_parallel_with(main_loop)
