#!/usr/bin/python3

import math
import rawpy

filename = '/home/Downloads/raw_default/raw_default/raw_default.dng'
threshold_lo = 7000
threshold_hi = 10000

image = rawpy.imread(filename)
raw = image.raw_image
bpp = int(math.log2(image.white_level + 1))
bayer_pattern = image.raw_pattern
channel = {}
mean = {}

for i in range(2):
    for j in range(2):
        channel[bayer_pattern[i, j]] = raw[i::2, j::2]

# Shift to 16-bits
for i in channel.keys():
    channel[i] = channel[i] << (16 - bpp)
    mean[i] = channel[i].mean()

# Normalise means
max_mean = max([m for m in mean.values()])
for i in channel.keys():
    channel[i] = channel[i] * max_mean / mean[i]

for i in range(4):
    c = channel[i].flatten()
    count_lo = len([s for s in c if s > threshold_lo])
    count_hi = len([s for s in c if s > threshold_hi])
    print(f'ch {i} : lo {count_lo} hi {count_hi} mean {mean[i]:.2f}')
