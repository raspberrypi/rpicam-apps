#!/usr/bin/python3

import math
import rawpy
import numpy

filename = 'C:\\Users\\CAM_JIG\\Desktop\\brightness_m12_13_400_2.dng'
threshold_lo = 10000
threshold_hi = 18000

image = rawpy.imread(filename)
raw = image.raw_image
bpp = int(math.log2(image.white_level + 1))
bayer_pattern = image.raw_pattern
channel = {}
mean = {}
min_val = {}

for i in range(2):
    for j in range(2):
        channel[bayer_pattern[i, j]] = raw[i::2, j::2]

# Shift to 16-bits
for i in channel.keys():
    channel[i] = channel[i] << (16 - bpp)
    mean[i] = channel[i].mean()
    min_val[i] = min(channel[i].flatten())

# Normalise means
max_mean = max([m for m in mean.values()])
for i in channel.keys():
    channel[i] = channel[i] * max_mean / mean[i]

print(f'\n{"-" * 10 } {filename} {"-" * 10 }')

for i in range(4):
    c = channel[i].flatten()
    count_lo = len([s for s in c if s <= threshold_lo])
    count_hi = len([s for s in c if s <= threshold_hi])
    print(f'ch {i} : lo {count_lo} hi {count_hi} mean {mean[i]:.2f} min {min_val[i]:.2f}')

ch = numpy.concatenate((channel[0], channel[1], channel[2], channel[3]))
bins = range(4000, 32768, 1000)
for b in bins:
    print(f'{b} : count {len([s for s in ch.flatten() if s <= b])}')
