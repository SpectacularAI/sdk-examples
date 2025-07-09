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

def get_position(vio_out):
    import numpy as np
    # supports two slightly different JSONL formats
    if 'pose' in vio_out: vio_out = vio_out['pose']
    # SDK < 0.12 does not expose the TRACKING status
    is_tracking = vio_out.get('status', 'TRACKING') == 'TRACKING'
    data = {}
    for c in 'xyz':
        val = vio_out['position'][c]
        if not is_tracking: val = np.nan
        data[c] = val
    return data

def make_plotter(initial_scale=1.0, center=(0, 0, 0)):
    import numpy as np
    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure()
    ax = Axes3D(fig, auto_add_to_figure=False)
    fig.add_axes(ax)

    lims = [(c - initial_scale/2, c + initial_scale/2) for c in center]
    ax.set(xlim=lims[0], ylim=lims[1], zlim=lims[2])
    ax.view_init(azim=-140) # initial plot orientation

    vio_plot = ax.plot(
        xs=[], ys=[], zs=[],
        linestyle="-",
        marker=""
    )
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")

    ax.set_title("VIO trajectory")

    data = { c: [] for c in 'xyz' }

    control = { 'close': False }
    fig.canvas.mpl_connect('close_event', lambda _: control.update({'close': True}))

    def update_data(vio_out):
        if control['close']: return False
        xyz = get_position(vio_out)
        for c, val in xyz.items():
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
    import argparse
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--dataFolder", help="Instead of running live mapping session, replay session from this folder")
    parser.add_argument('--file', type=argparse.FileType('r'),
        help='Read data from a JSONL file or pipe instead of displaying it live',
        default=None)
    parser.add_argument('--initialScale', type=float, default=1.0,
        help="Initial size of the figure in meters")

    args = parser.parse_args()

    # auto-scale figure with file inputs
    scale = args.initialScale
    center = (0, 0, 0)
    data_loaded_from_file = []
    if args.file:
        import numpy as np
        data = { c: [] for c in 'xyz' }
        for vio_out in file_vio_reader(args.file):
            data_loaded_from_file.append(vio_out)
            pos = get_position(vio_out)
            if not np.isnan(pos['x']):
                for c, v in pos.items():
                    if not np.isnan(v):
                        data[c].append(v)

        if len(data['x']) > 1:
            scale = max([(np.max(data[c]) - np.min(data[c])) for c in 'xyz'] + [1e-10])
            center = [(np.max(data[c]) + np.min(data[c]))/2 for c in 'xyz']

    plotter, anim = make_plotter(initial_scale=scale, center=center)

    def reader_loop():
        replay = None
        if args.dataFolder:
            import spectacularAI
            replay = spectacularAI.Replay(args.dataFolder)
            vio_source = replay_vio_reader(replay)
        elif args.file:
            def in_memory_vio_reader():
                for vio_out in data_loaded_from_file:
                    yield(vio_out)
            vio_source = in_memory_vio_reader()
        else:
            vio_source = live_vio_reader()

        for vio_out in vio_source:
            if not plotter(vio_out): break
        if replay: replay.close()

    reader_thread = threading.Thread(target = reader_loop)
    reader_thread.start()
    plt.show()
    reader_thread.join()

