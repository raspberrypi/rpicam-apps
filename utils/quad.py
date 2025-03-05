#!/usr/bin/python3

import math
import rawpy
import numpy as np
import sys
import random

filename = sys.argv[1] if len(sys.argv) > 1 else 'please_specify_filename.dng'
black_level = 4096
viz = False

def fit_line(a):
    MARGIN2 = 36.0
    MAX_GRADIENT = 0.26
    best_score = 0.0;
    params = [ 0.0, 0.0 ]
    exes = np.arange(0, len(a), dtype=float)

    for x0 in range(len(a)//3, 2*len(a)//3):
        if (a[x0]==0):
            continue
        for guess in range(10):
            x1 = random.randint(len(a)//3,2*len(a)//3)
            if (abs(x1-x0) < 64 or a[x1]==0):
                continue
            p1 = float(a[x1]-a[x0]) / float(x1 - x0);
            p0 = float(a[x0]) - p1*x0;
        if (abs(p1) > MAX_GRADIENT):
            continue

        errs = exes*p1 + p0 - a
        score = np.sum(np.where(a != 0, np.maximum(0.0, MARGIN2 - np.square(errs)), 0.0))
        if (score > best_score):
            best_score = score
            params = [ p0, p1 ]

    if best_score < 4.0*MARGIN2:
        raise RuntimeError("Line not found")
    return params

def find_centile(histogram, start, finish, centile):
    tot = np.sum(histogram[start:finish], dtype=float)
    thresh = (tot * centile) / 100.0
    cum = 0.0
    for i in range(start,finish):
        cum += histogram[i]
        if (cum >= thresh):
            return i
    return finish

print(f'\n{"-" * 10 } {filename} {"-" * 10 }')
image = rawpy.imread(filename)
raw = image.raw_image
bpp = int(math.log2(image.white_level + 1))

if (len(raw) > 2000 or len(raw[0]) > 3200):
    print(f'Resizing (binning) the input')
    cpt0 = (raw[0::4,0::4] + raw[0::4, 2::4] + raw[2::4,0::4] + raw[2::4,2::4])
    cpt1 = (raw[0::4,1::4] + raw[0::4, 3::4] + raw[2::4,1::4] + raw[2::4,3::4])
    cpt2 = (raw[1::4,0::4] + raw[1::4, 2::4] + raw[3::4,0::4] + raw[3::4,2::4])
    cpt3 = (raw[1::4,1::4] + raw[1::4, 3::4] + raw[3::4,1::4] + raw[3::4,3::4])
    raw = np.zeros((len(raw)//2, len(raw[0])//2), int)
    raw[0::2,0::2] = cpt0
    raw[0::2,1::2] = cpt1
    raw[1::2,0::2] = cpt2
    raw[1::2,1::2] = cpt3
    del cpt3
    del cpt2
    del cpt1
    del cpt0
    bpp += 2

raw <<= (16 - bpp)
height = len(raw)
width = len(raw[0])
print(f'Width: {width}, Height: {height}')

# Fail if more than 1% of pixels are likely to have saturated.
bright_count = sum(raw.flatten() >= 0xFE00)
print(f'Saturation fraction: {100.0*bright_count / float(width*height):4.2f}%')
if (100 * bright_count > width * height):
    raise RuntimeError("Image too bright for Quadrant test")

# Find the largest H step (of any colour component) in the middle third of
# each scanline. Compute sums over pairs of rows, with black level subtracted.
tmp = np.absolute(np.apply_along_axis(lambda m: np.convolve(m, np.array([-1, 0, -2, 0, -3, 0, -3, 0, -3, 0, 3, 0, 3, 0, 3, 0, 2, 0, 1]), 'same'), axis=1, arr=raw))
steps = np.argmax(tmp[:,(width//3 - 2):(2*width//3 + 2)], axis=1) + width//3 - 2
steps = np.where(np.logical_and(steps >= width//3, steps < 2*width//3), steps, 0)
rowsums = np.sum(raw, axis=1, dtype=int)
rowsums = np.maximum(rowsums - width * black_level, 0)
rowsums = np.array(rowsums[0::2]) + np.array(rowsums[1::2])

# Try to find the vertical line.
vline_params = fit_line(steps)

# Find the largest V step (of any colour component) in the middle third of
# each column. Compute sums over pairs of colums, with black level subtracted.
tmp = np.absolute(np.apply_along_axis(lambda m: np.convolve(m, np.array([-1, 0, -2, 0, -3, 0, -3, 0, -3, 0, 3, 0, 3, 0, 3, 0, 2, 0, 1]), 'same'), axis=0, arr=raw))
steps = np.argmax(tmp[(height//3 - 2):(2*height//3 + 2),:], axis=0) + height//3 - 2
steps = np.where(np.logical_and(steps >= height//3, steps < 2*height//3), steps, 0)
colsums = np.sum(raw, axis=0, dtype=int)
colsums = np.maximum(colsums - height * black_level, 0)
colsums = np.array(colsums[0::2]) + np.array(colsums[1::2])

# Try to find the horizontal line. Check the crossing angle is 90 +/- 5 degrees or so.
hline_params = fit_line(steps)
if (abs(hline_params[1] + vline_params[1]) > 0.088):
    raise RuntimeError("Detected lines not at right angles")

# Find the intersection point. Both lines have to pass through the central
# third of the width and of the height, but not necessarily to intersect there.
# Let's require them to intersect in the central half of the width and height.
intersect_x = (int)(0.5 + (vline_params[0] + vline_params[1] * hline_params[0]) /
                    (1.0 - vline_params[1] * hline_params[1]))
intersect_y = (int)(0.5 + (hline_params[0] + hline_params[1] * vline_params[0]) /
                    (1.0 - vline_params[1] * hline_params[1]));
print(f'Lines intersect at {intersect_x},{intersect_y}')
if (4*intersect_x < width or 4*intersect_x >= 3 * width or
    4*intersect_y < height or 4*intersect_y >= 3 * height):
    raise RuntimeError("Intersection point not central")

# Find the bounding box containing 75% of the light in each of the 4 principal directions
# from the intersection (with a 64 pixel margin, but no attempt to correct for rotation).
# NB: bounding box coordinates are always even, so they align with pixel-quads.
vbounds = [ 2 * find_centile(rowsums, 0,                     (intersect_y - 64)//2, 25),
            2 * find_centile(rowsums, (intersect_y + 65)//2, height//2,             75) ]
hbounds = [ 2 * find_centile(colsums, 0,                     (intersect_x - 64)//2, 25),
            2 * find_centile(colsums, (intersect_x + 65)//2, width//2,              75) ]

# Select pixel-quads that are within the 75% bounding box, more than 64 pel away from
# a boundary line (this time, corrected for rotation) and not saturated; these will be
# used to estimate the mean colour of each quadrant.
xpositions = np.tile(np.linspace(0,width,num=width//2,endpoint=False),(height//2,1))
ypositions = np.tile(np.linspace(0,height,num=height//2, endpoint=False)[np.newaxis].transpose(), (1,width//2))
hoffsets   = xpositions - vline_params[0] - ypositions *vline_params[1]
voffsets   = ypositions - hline_params[0] - xpositions *hline_params[1]
quadrant   = 2*(voffsets > 0) + 1*(hoffsets > 0)
validity   = np.logical_and(np.logical_and(ypositions >= vbounds[0], ypositions < vbounds[1]),
                            np.logical_and(xpositions >= hbounds[0], xpositions < hbounds[1]))
del ypositions
del xpositions
validity   = np.logical_and(validity,
                            np.logical_and(np.logical_or(hoffsets < -64.0, hoffsets >= 64.0),
                                           np.logical_or(voffsets < -64.0, voffsets >= 64.0)))
cpts       = [ raw[0::2,0::2], raw[0::2,1::2], raw[1::2,0::2], raw[1::2,1::2] ]
validity   = np.logical_and(validity,
                            np.maximum(np.maximum(cpts[0], cpts[1]), np.maximum(cpts[2], cpts[3])) < 0xFE00)

# Report mean colours. For compatibility with cal/dust test output, print them WITH black level.
# Also report the fraction of light in each Bayer component (NOT including black level).
count = [ 0, 0, 0, 0 ]
mean = np.zeros((4,4), float)
for q in range(4):
    tot = 0
    count[q] = np.sum(np.logical_and(validity, quadrant == q))
    mean[q] = [ np.sum(np.logical_and(validity, quadrant == q) * cpts[0]),
                np.sum(np.logical_and(validity, quadrant == q) * cpts[1]),
                np.sum(np.logical_and(validity, quadrant == q) * cpts[2]),
                np.sum(np.logical_and(validity, quadrant == q) * cpts[3]) ]
    mean[q] = (mean[q] / count[q]) - black_level
    tot += sum(mean[q])
    print(f'Means: [ {int(mean[q][0]+black_level):5d}, {int(mean[q][1]+black_level):5d}, {int(mean[q][2]+black_level):5d}, {int(mean[q][3]+black_level):5d} ] Fractions: [ {mean[q,0]/tot:5.3f}, {mean[q,1]/tot:5.3f}, {mean[q,2]/tot:5.3f}, {mean[q,3]/tot:5.3f} ]')

# Final image pass. Try to estimate the fraction of the image covered by the test pattern,
# by counting pix-quads whose normalized correlation with the quadrant's mean colour is >= 1/8
for q in range(4):
    mag2 = sum(i*i for i in mean[q])
    mean[q] = mean[q] / mag2

means = [ np.choose(quadrant, (mean[0][0], mean[1][0], mean[2][0], mean[3][0])),
          np.choose(quadrant, (mean[0][1], mean[1][1], mean[2][1], mean[3][1])),
          np.choose(quadrant, (mean[0][2], mean[1][2], mean[2][2], mean[3][2])),
          np.choose(quadrant, (mean[0][3], mean[1][3], mean[2][3], mean[3][3])) ]

cpts[2] = (np.multiply(means[0],cpts[0] - black_level) + np.multiply(means[1],cpts[1] - black_level) + np.multiply(means[2],cpts[2] - black_level) + np.multiply(means[3],cpts[3] - black_level) >= 0.125)
cpts[2] = np.logical_and(cpts[2], np.c_[cpts[2][:,1:], np.ones(height//2,dtype=bool)])
print(f'Area factor: {400.0*np.sum(cpts[2]) / float(width*height):4.1f}%')

# Debug visualization
if (viz):
    from matplotlib import pyplot as plt
    vizimg = np.dstack((np.sqrt(cpts[3]),np.sqrt(cpts[1]),np.sqrt(cpts[0]))).astype(int)
    vizimg[:,:,0] = np.where(validity, vizimg[:,:,0]//2, vizimg[:,:,0]) # mark sample patches as darker
    vizimg[:,:,1] = np.where(validity, vizimg[:,:,1]//2, vizimg[:,:,1])
    vizimg[:,:,2] = np.where(validity, vizimg[:,:,2]//2, vizimg[:,:,2])
    vizimg[:,:,1] = np.where(cpts[2], vizimg[:,:,1], 128+vizimg[:,:,1]//2) # mark dark pixels as bright green
    vizimg[:,:,1] = np.where(np.logical_and(np.abs(hoffsets) > 1.0, np.abs(voffsets) > 1.0), vizimg[:,:,1], 0xFF) # draw lines in green
    for i in range(len(rowsums)):
        vizimg[i,(rowsums[i]>>18),0] = 255
    for i in range(len(colsums)):
        vizimg[height//2-1-(colsums[i]>>18),i,2] = 255
    plt.imshow(vizimg)
    plt.show()
