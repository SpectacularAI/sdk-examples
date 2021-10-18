"""
Simple VIO result visualizer Python. Reads outputs from the
Spectacular AI OAK-D plugin and plots them in real time.

Plug in the OAK-D to an USB3 port using an USB3 cable before running.
"""
import time
import json
import threading
import matplotlib.pyplot as plt
import depthai
import spectacularAI

def read_vio():
    pipeline = depthai.Pipeline()
    vio_pipeline = spectacularAI.depthai.Pipeline(pipeline)

    with depthai.Device(pipeline) as device:
        vio_session = vio_pipeline.startSession(device)

        while True:
            if vio_session.hasOutput():
                out = vio_session.getOutput()
                yield(out)
            else:
                if not vio_session.work():
                    time.sleep(0.005)

def make_plotter():
    import numpy as np
    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure()
    ax = Axes3D(fig)
    fig.add_axes(ax)

    ax_bounds = (-0.5, 0.5) # meters
    ax.set(xlim=ax_bounds, ylim=ax_bounds, zlim=ax_bounds)
    ax.view_init(azim=-140) # initial plot orientation

    vio_plot = ax.plot(
        xs=[], ys=[], zs=[],
        linestyle="-",
        marker="",
        #label='VIO trajectory',
    )
    #ax.legend()
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")

    title = ax.set_title("VIO trajectory")

    data = { c: [] for c in 'xyz' }

    control = { 'close': False }
    fig.canvas.mpl_connect('close_event', lambda _: control.update({'close': True}))

    def update_data(vio_out):
        if control['close']: return False
        for c in 'xyz':
            data[c].append(getattr(vio_out.pose.position, c))
        return True

    def update_graph(frames):
        x, y, z = [np.array(data[c]) for c in 'xyz']
        vio_plot[0].set_data(x, y)
        vio_plot[0].set_3d_properties(z)
        # plt.draw()
        # plt.pause(0.01)
        return (vio_plot[0],)

    from matplotlib.animation import FuncAnimation
    anim = FuncAnimation(fig, update_graph, interval=15, blit=True)
    return update_data, anim

if __name__ == '__main__':
    plotter, anim = make_plotter()

    def reader_loop():
        for vio_out in read_vio():
            # print('\t'.join(['%+.04f' % vio_out['position'][c] for c in 'xyz']))
            if not plotter(vio_out): break

    reader_thread = threading.Thread(target = reader_loop)
    reader_thread.start()
    plt.show()
    reader_thread.join()
