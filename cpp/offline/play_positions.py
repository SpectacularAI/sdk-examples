#!/usr/bin/env python3

"""Play position trajectory and SDK statuses from JSONL output."""

import bisect
import json
import sys
import time

import numpy as np

POSITION = 'position'
TIME = 'time'
STATUS = 'status'

STATUS_COLORS = {
    'TRUE': '#1a9850',
    'FALSE': '#d73027',
    'NA': '#888888',
    'DEPRECATED': '#888888',
}


def read_jsonl(fn):
    with open(fn) as f:
        for l in f:
            yield json.loads(l)


def read_data(fn):
    times = []
    pos = []
    status_times = []
    statuses = []
    cur_time = 0.0

    for o in read_jsonl(fn):
        if TIME in o and POSITION in o:
            cur_time = float(o[TIME])
            times.append(cur_time)
            pos.append([o[POSITION][c] for c in 'xyz'])
        elif STATUS in o and isinstance(o[STATUS], dict):
            if TIME in o:
                cur_time = float(o[TIME])
            status_times.append(cur_time)
            statuses.append(o[STATUS])

    if times and status_times and status_times[0] == 0.0:
        status_times[0] = times[0]

    return np.array(times), np.array(pos), status_times, statuses


def set_axes_equal_3d(ax):
    from mpl_toolkits.mplot3d.axes3d import Axes3D
    assert isinstance(ax, Axes3D)
    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])  # type: ignore[attr-defined]
    center = limits.mean(axis=1)
    half_range = max((limits[:, 1] - limits[:, 0]).max() / 2, 1e-3)
    ax.set_xlim3d(center[0] - half_range, center[0] + half_range)
    ax.set_ylim3d(center[1] - half_range, center[1] + half_range)
    ax.set_zlim3d(center[2] - half_range, center[2] + half_range)


if __name__ == '__main__':
    import argparse
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('jsonl', help='VIO output file')
    p.add_argument('speed', nargs='?', type=float, default=1.0, help='Playback speed factor (default: 1.0)')
    p.add_argument('--3d', action='store_true', dest='plot3d', help='Plot in 3D (interactively rotatable)')
    args = p.parse_args()

    times, pos, status_times, statuses = read_data(args.jsonl)
    if pos.size == 0:
        print("No data to plot.")
        sys.exit()

    t0 = times[0]
    t_end = times[-1]
    duration = max(t_end - t0, 0.0)

    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.1, 1.4], wspace=0.08)

    if args.plot3d:
        from mpl_toolkits.mplot3d.axes3d import Axes3D
        ax_plot = fig.add_subplot(gs[0, 0], projection='3d')
        assert isinstance(ax_plot, Axes3D)
        (line,) = ax_plot.plot([], [], [])
        (head,) = ax_plot.plot([], [], [], marker='o', markersize=5, color='C0')
        ax_plot.set_xlabel('x')
        ax_plot.set_ylabel('y')
        ax_plot.set_zlabel('z')
    else:
        ax_plot = fig.add_subplot(gs[0, 0])
        (line,) = ax_plot.plot([], [])
        (head,) = ax_plot.plot([], [], marker='o', markersize=5, color='C0')
        ax_plot.set_aspect('equal', adjustable='datalim')

    ax_status = fig.add_subplot(gs[0, 1])
    ax_status.axis('off')

    flag_names = list(statuses[0].keys()) if statuses else []
    status_texts = {}
    if flag_names:
        n_cols = 2 if len(flag_names) > 30 else 1
        n_rows = (len(flag_names) + n_cols - 1) // n_cols
        row_step = min(0.036, 0.96 / max(n_rows, 1))
        for idx, name in enumerate(flag_names):
            col = idx // n_rows
            row = idx % n_rows
            x_val = col * 0.50
            y = 0.98 - row * row_step
            val_text = ax_status.text(
                x_val, y, '',
                transform=ax_status.transAxes,
                fontfamily='monospace',
                fontsize=14,
                fontweight='bold',
                verticalalignment='top',
            )
            ax_status.text(
                x_val, y, f"{'':12}{name}",
                transform=ax_status.transAxes,
                fontfamily='monospace',
                fontsize=14,
                verticalalignment='top',
            )
            status_texts[name] = val_text

    state = {
        'last_wall': None,
        'elapsed_sim': 0.0,
        'paused': False,
        'last_status_idx': -1,
    }

    def set_title(cur_t):
        paused_str = " [PAUSED]" if state['paused'] else ""
        ax_plot.set_title(f"Time: {cur_t - t0:.1f}s / {duration:.1f}s ({args.speed:g}x){paused_str}")

    def on_key(event):
        if event.key != 'a':
            return
        now = time.monotonic()
        if state['paused']:
            state['paused'] = False
            if state['last_wall'] is not None:
                state['last_wall'] = now
        else:
            if state['last_wall'] is not None:
                state['elapsed_sim'] += (now - state['last_wall']) * args.speed
                state['last_wall'] = now
            state['paused'] = True
        set_title(min(t0 + state['elapsed_sim'], t_end))
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect('key_press_event', on_key)

    def update(_frame):
        now = time.monotonic()
        if state['last_wall'] is None:
            state['last_wall'] = now
        elif state['paused']:
            return []
        else:
            state['elapsed_sim'] += (now - state['last_wall']) * args.speed
            state['last_wall'] = now

        cur_t = min(t0 + state['elapsed_sim'], t_end)

        pos_idx = int(np.searchsorted(times, cur_t, side='right'))
        if pos_idx > 0:
            cur_pos = pos[:pos_idx]
            if args.plot3d:
                line.set_data(cur_pos[:, 0], cur_pos[:, 1])
                line.set_3d_properties(cur_pos[:, 2])  # type: ignore[attr-defined]
                head.set_data([cur_pos[-1, 0]], [cur_pos[-1, 1]])
                head.set_3d_properties([cur_pos[-1, 2]])  # type: ignore[attr-defined]
                ax_plot.set_xlim3d(cur_pos[:, 0].min(), cur_pos[:, 0].max() + 1e-6)  # type: ignore[attr-defined]
                ax_plot.set_ylim3d(cur_pos[:, 1].min(), cur_pos[:, 1].max() + 1e-6)  # type: ignore[attr-defined]
                ax_plot.set_zlim3d(cur_pos[:, 2].min(), cur_pos[:, 2].max() + 1e-6)  # type: ignore[attr-defined]
                set_axes_equal_3d(ax_plot)
            else:
                line.set_data(cur_pos[:, 0], cur_pos[:, 1])
                head.set_data([cur_pos[-1, 0]], [cur_pos[-1, 1]])
                ax_plot.relim()
                ax_plot.autoscale_view()

        if statuses:
            s_idx = bisect.bisect_right(status_times, cur_t) - 1
            if s_idx < 0:
                s_idx = 0
            if s_idx != state['last_status_idx']:
                state['last_status_idx'] = s_idx
                cur_status = statuses[s_idx]
                for name, val_text in status_texts.items():
                    val = cur_status.get(name, '')
                    val_text.set_text(val)
                    val_text.set_color(STATUS_COLORS.get(val, '#000000'))

        set_title(cur_t)
        if cur_t >= t_end:
            anim.event_source.stop()

        return []

    anim = FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()
