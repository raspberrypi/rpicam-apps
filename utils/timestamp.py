#!/usr/bin/python3
#
# libcamera-apps timestamp analysis tool
# Copyright (C) 2021, Raspberry Pi Ltd.
#
import argparse
import json
import subprocess

try:
    from matplotlib import pyplot as plt
    plot_available = True
except ImportError:
    plot_available = False


def read_times_pts(file):
    with open(file) as f:
        if f.readline().strip() != '# timecode format v2':
            raise RuntimeError('PTS file format unknown')
        return [float(line) for line in f.readlines()]


def read_times_container(file):
    cmd = ['ffprobe', file, '-hide_banner', '-select_streams', 'v', '-show_entries', 'frame', '-of', 'json']
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    if r.returncode:
        raise RuntimeError(f'ffprobe failed to run with command:\n{" ".join(cmd)}')

    frame_data = json.loads(r.stdout)['frames']
    keys = ['pkt_pts_time', 'pts_time', 'pkt_dts_time', 'dts_time']
    key = [k for k in keys if k in frame_data[0].keys()]

    if len(key) == 0:
        raise RuntimeError(f'Timestamp keys not found in {file}')

    ts_list = [float(f[key[0]]) * 1000 for f in frame_data]
    return ts_list


def get_differences(items):
    return [next_item - item for item, next_item in zip(items[:-1], items[1:])]


def outliers(diffs, frac, avg):
    return f'{sum(d < (1 - frac) * avg or d > (1 + frac) * avg for d in diffs)} ({frac * 100}%)'


def plot_pts(diffs, avg, title):
    fig, ax = plt.subplots()
    ax.plot(diffs, label='Frame times')
    ax.plot([0, len(diffs)], [avg, avg], 'g--', label='Average')
    # Find an plot the max value
    max_val, idx = max((val, idx) for (idx, val) in enumerate(diffs))
    ax.plot([idx], [max_val], 'rx', label='Maximum')
    ax.axis([0, len(diffs), min(diffs) * 0.995, max_val * 1.005])
    ax.legend()
    plt.title(title)
    plt.xlabel('Frame')
    plt.ylabel('Frame time (ms)')
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='libcamera-apps timestamp analysis tool')
    parser.add_argument('filename', help='PTS file generated from libcamera-vid (with a .txt or .pts extension)'
                                         ' or an avi/mkv/mp4 container file', type=str)
    parser.add_argument('--plot', help='Plot timestamp graph', action='store_true')
    args = parser.parse_args()

    if args.filename.lower().endswith(('.txt', '.pts')):
        times = read_times_pts(args.filename)
    elif args.filename.lower().endswith(('.avi', '.mkv', '.mp4')):
        times = read_times_container(args.filename)
    else:
        raise RuntimeError('Unknown file format')

    diffs = get_differences(times)
    avg = sum(diffs) / len(diffs)
    min_val, min_idx = min((val, idx) for (idx, val) in enumerate(diffs))
    max_val, max_idx = max((val, idx) for (idx, val) in enumerate(diffs))
    print(f'Total: {len(diffs) + 1} frames ({len(diffs)} samples)')
    print(f'Average: {avg:.3f} ms / {1e3/avg:.3f} fps')
    print(f'Minimum: {min_val:.3f} ms at frame {min_idx}')
    print(f'Maximum: {max_val:.3f} ms at frame {max_idx}')
    print('Outliers:', *[outliers(diffs, f, avg) for f in (1, .1, .01, .001)])

    if args.plot:
        if plot_available:
            plot_pts(diffs, avg, f'{args.filename}')
        else:
            print('\nError: matplotlib is not installed, please install with "pip3 install matplotlib"')
