#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Louis Brooks

"""Convert raw 1 KB SSD1306 frame dumps (*.bin, 128x64, page-major, LSB = top pixel) to magnified PNGs.
usage: topng.py <dir_with_bins> <output_dir>
"""
import sys, glob, os
from PIL import Image

SCALE = 5
def load(path):
    b = open(path, 'rb').read()
    assert len(b) == 1024, path
    im = Image.new('L', (128, 64), 0)
    px = im.load()
    for page in range(8):
        for x in range(128):
            v = b[page * 128 + x]
            for bit in range(8):
                if v & (1 << bit):
                    px[x, page * 8 + bit] = 255
    return im

def tint(im):
    im = im.resize((128 * SCALE, 64 * SCALE), Image.NEAREST).convert('RGB')
    r, g, b = im.split()
    return Image.merge('RGB', (r.point(lambda v: int(v * 0.35)), g.point(lambda v: int(v * 0.8)), b))

src, dst = sys.argv[1], sys.argv[2]
os.makedirs(dst, exist_ok=True)
for f in sorted(glob.glob(os.path.join(src, '*.bin'))):
    name = os.path.splitext(os.path.basename(f))[0]
    out = tint(load(f))
    canvas = Image.new('RGB', (out.width + 16, out.height + 16), (24, 24, 24))
    canvas.paste(out, (8, 8))
    canvas.save(os.path.join(dst, name + '.png'))
    print('wrote', name + '.png')
