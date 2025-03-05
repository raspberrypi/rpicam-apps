#!/usr/bin/python3

import math
import rawpy
import cv2
import sys

if len(sys.argv) < 3:
    print(f'Usage: {sys.argv[0]} <filename> <threshold>')
    sys.exit(-1)

filename = sys.argv[1]
threshold = int(sys.argv[2])

image = rawpy.imread(filename)
raw = image.raw_image
bpp = int(math.log2(image.white_level + 1))
raw = raw << (16 - bpp)

raw[raw < threshold] = 65535

cv2.imshow(f'Threshold {threshold}', raw)
cv2.waitKey(0)
cv2.destroyAllWindows()
