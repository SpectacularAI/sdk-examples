"""
Simple VIO result visualizer Python. Reads outputs from the
Spectacular AI OAK-D plugin and plots them in real time.
Plug in the OAK-D to an USB3 port using an USB3 cable before running.

Can also visualize pre-recorded results using Replay API, or from a JSONL file or from a pipe.
The device does not have to be attached in this case. (See vio_record.py)
"""
import time
import json
import threading
import matplotlib.pyplot as plt

def live_vio_reader():
    import depthai
    import spectacularAI
    pipeline = depthai.Pipeline()
    vio_pipeline = spectacularAI.depthai.Pipeline(pipeline)

    with depthai.Device(pipeline) as device, \
        vio_pipeline.startSession(device) as vio_session:

        while True:
            out = vio_session.waitForOutput()
            yield(json.loads(out.asJson()))

def replay_vio_reader(replay):
    outputs = []
    def onOutput(out):
        outputs.append(out.asJson())

    replay.setOutputCallback(onOutput)
    replay.startReplay()

    while True:
        if outputs:
            out = outputs.pop(0)
            yield(json.loads(out))
        time.sleep(0.01)

def file_vio_reader(in_stream):
    while True:
        line = in_stream.readline()
        if not line: break
        try:
            d = json.loads(line)
            if 'position' not in d and 'pose' not in d: continue
            yield(d)
        except:
            # Ignore all lines that aren't valid json
            pass

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
        marker=""
    )
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")

    title = ax.set_title("VIO trajectory")

    data = { c: [] for c in 'xyz' }

    control = { 'close': False }
    fig.canvas.mpl_connect('close_event', lambda _: control.update({'close': True}))

    def update_data(vio_out):
        if control['close']: return False
        # supports two slightly different JSONL formats
        if 'pose' in vio_out: vio_out = vio_out['pose']
        # SDK < 0.12 does not expose the TRACKING status
        is_tracking = vio_out.get('status', 'TRACKING') == 'TRACKING'
        for c in 'xyz':
            val = vio_out['position'][c]
            if not is_tracking: val = np.nan
            data[c].append(val)
        return True

    def update_graph(frames):
        x, y, z = [np.array(data[c]) for c in 'xyz']
        vio_plot[0].set_data(x, y)
        vio_plot[0].set_3d_properties(z)
        return (vio_plot[0],)

    from matplotlib.animation import FuncAnimation
    anim = FuncAnimation(fig, update_graph, interval=15, blit=True)
    return update_data, anim

if __name__ == '__main__':
    plotter, anim = make_plotter()
    import argparse
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--dataFolder", help="Instead of running live mapping session, replay session from this folder")
    parser.add_argument('--file', type=argparse.FileType('r'),
        help='Read data from a JSONL file or pipe instead of displaying it live',
        default=None)

    args = parser.parse_args()

    def reader_loop():
        replay = None
        if args.dataFolder:
            import spectacularAI
            replay = spectacularAI.Replay(args.dataFolder)
            vio_source = replay_vio_reader(replay)
        elif args.file:
            vio_source = file_vio_reader(args.file)
        else:
            vio_source = live_vio_reader()

        for vio_out in vio_source:
            if not plotter(vio_out): break
        if replay: replay.close()

    reader_thread = threading.Thread(target = reader_loop)
    reader_thread.start()
    plt.show()
    reader_thread.join()

