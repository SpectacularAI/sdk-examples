"""
Draw in the air: cover the OAK-D color camera to activate the ink.
Requirements: pip install matplotlib opencv-python
"""
import depthai
import time
import matplotlib.pyplot as plt
import spectacularAI
import threading
import numpy as np

SHOW_CAM = True
if SHOW_CAM:
    import cv2

def make_pipelines():
    pipeline = depthai.Pipeline()
    vio_pipeline = spectacularAI.depthai.Pipeline(pipeline)

    RGB_OUTPUT_WIDTH = 200 # very small on purpose
    REF_ASPECT = 1920 / 1080.0
    w = RGB_OUTPUT_WIDTH
    h = int(round(w / REF_ASPECT))

    camRgb = pipeline.createColorCamera()
    camRgb.setPreviewSize(w, h)
    camRgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setColorOrder(depthai.ColorCameraProperties.ColorOrder.BGR)
    camRgb.initialControl.setAutoFocusMode(depthai.RawCameraControl.AutoFocusMode.OFF)
    camRgb.initialControl.setManualFocus(0) # seems to be about 1m, not sure about the units
    camRgb.initialControl.setManualExposure(10000, 1000)
    out_source = camRgb.preview

    xout_camera = pipeline.createXLinkOut()
    xout_camera.setStreamName("rgb")
    out_source.link(xout_camera.input)

    return pipeline, vio_pipeline

def make_camera_wireframe(aspect=640/400., scale=0.0025):
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
    def __init__(self):
        from mpl_toolkits.mplot3d import Axes3D
        from matplotlib.animation import FuncAnimation

        self.ink_active = False
        self.prev_ink_active = False

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

        ax.legend()
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_zlabel("z (m)")

        def on_close(*args):
            self.should_close = True

        fig.canvas.mpl_connect('close_event', on_close)

        self.cam_wire = make_camera_wireframe()
        self.vio_data = vio_data
        self.vio_cam_data = vio_cam_data
        self.should_close = False

        def update_graph(*args):
            r = []
            for graph in [self.vio_data, self.vio_cam_data]:
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
            if self.ink_active:
                self.vio_data[c].append(getattr(vio_out.pose.position, c))
            elif not self.prev_ink_active:
                # NaN can be used to break lines in matplotlib
                self.vio_data[c].append(np.nan)

        self.prev_ink_active = self.ink_active

        return True

    def set_ink_active(self, active):
        self.ink_active = active

    def clear(self):
        self.ink_active = False
        for c in 'xyz': del self.vio_data[c][:]

    def start_in_parallel_with(self, parallel_thing):
        thread = threading.Thread(target = parallel_thing)
        thread.start()
        plt.show()
        thread.join()

if __name__ == '__main__':
    pipeline, vio_pipeline = make_pipelines()

    with depthai.Device(pipeline) as device, \
        vio_pipeline.startSession(device) as vio_session:

        visu_3d = MatplotlibVisualization()

        def main_loop():
            rgbQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

            lightness = 1.0
            while True:
                if vio_session.hasOutput():
                    vio_out = vio_session.getOutput()
                    if not visu_3d.update_vio(vio_out): break
                elif rgbQueue.has():
                    rgbFrame = rgbQueue.get()

                    lightness = np.max(rgbFrame.getRaw().data) / 255.0
                    # print(lightness)

                    THRESHOLD = 0.6
                    visu_3d.set_ink_active(lightness < THRESHOLD)

                    if SHOW_CAM:
                        cv2.imshow("rgb", rgbFrame.getCvFrame())

                        cv_key = cv2.waitKey(1)
                        if cv_key == ord('q'):
                            break
                        elif cv_key == ord('c'): # for clear
                            visu_3d.clear()
                else:
                    time.sleep(0.005)

        visu_3d.start_in_parallel_with(main_loop)
