#!/usr/bin/python3
#
# libcamera-apps timestamp analysis tool
# Copyright (C) 2021, Raspberry Pi Ltd.
#
import argparse

try:
    from matplotlib import pyplot as plt
    plot_available = True
except ImportError:
    plot_available = False


def read_times(file):
    with open(file) as f:
        f.readline()  # there's one header line we must skip
        return [float(line) for line in f.readlines()]


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
    parser.add_argument('filename', help='PTS file generated from libcamera-vid', type=str)
    parser.add_argument('--plot', help='Plot timestamp graph', action='store_true')
    args = parser.parse_args()

    times = read_times(args.filename)
    diffs = get_differences(times)
    avg = sum(diffs) / len(diffs)
    min_val, min_idx = min((val, idx) for (idx, val) in enumerate(diffs))
    max_val, max_idx = max((val, idx) for (idx, val) in enumerate(diffs))
    print(f'Minimum: {min_val:.3f} ms at frame {min_idx}\nMaximum: {max_val:.3f} ms at frame {max_idx}\nAverage: {avg:.3f} ms')
    print(f'Total: {len(diffs)} samples')
    print('Outliers:', *[outliers(diffs, f, avg) for f in (1, .1, .01, .001)])

    if args.plot:
        if plot_available:
            plot_pts(diffs, avg, f'{args.filename}')
        else:
            print('\nError: matplotlib is not installed, please install with "pip3 install matplotlib"')
