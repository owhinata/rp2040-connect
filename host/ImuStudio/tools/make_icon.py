#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Draws the application icon: a tilted cube with its three axes.

    python3 tools/make_icon.py src/ImuStudio.App/Assets/icon.png

Needs Pillow. The result is committed; run this only to change the icon.
"""
import math
import sys

from PIL import Image, ImageDraw

SIZE = 256
SCALE = 4                     # drawn larger and reduced, for smooth edges
N = SIZE * SCALE


def rotation(yaw, pitch, roll):
    cy, sy, cp, sp, cr, sr = (f(a) for a in (yaw, pitch, roll) for f in (math.cos, math.sin))
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


R = rotation(math.radians(-40), math.radians(-8), math.radians(12))


def project(p):
    """Body point to pixels: rotate, then look from the front, a little above."""
    x, y, z = (sum(R[i][j] * p[j] for j in range(3)) for i in range(3))
    tilt = math.radians(36)
    u = y
    v = z * math.cos(tilt) - x * math.sin(tilt)
    depth = x * math.cos(tilt) + z * math.sin(tilt)
    return (N * (0.5 + 0.215 * u), N * (0.55 - 0.215 * v)), depth


def main(path):
    image = Image.new('RGBA', (N, N), (0, 0, 0, 0))
    draw = ImageDraw.Draw(image)
    margin = N * 0.04
    draw.rounded_rectangle((margin, margin, N - margin, N - margin), radius=N * 0.2, fill=(27, 32, 44, 255))

    faces = []                # (depth, outline, shade)
    for axis in range(3):
        for sign in (-1, 1):
            corners = []
            for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                p = [0.0, 0.0, 0.0]
                p[axis] = sign
                p[(axis + 1) % 3] = a
                p[(axis + 2) % 3] = b
                corners.append(project(p))
            normal = [0.0, 0.0, 0.0]
            normal[axis] = sign
            (_, depth) = project(normal)
            if depth > 0:     # faces towards the viewer
                light = sum(R[i][axis] * sign * l for i, l in enumerate((0.45, -0.35, 0.82)))
                faces.append((depth, [c[0] for c in corners], 150 + int(95 * max(0.0, min(1.0, light)))))
    for _, outline, shade in sorted(faces):
        draw.polygon(outline, fill=(shade, shade, min(255, shade + 6), 255))
        draw.line(outline + [outline[0]], fill=(27, 32, 44, 255), width=N // 110, joint='curve')

    # The axes leave the cube through the middle of its visible faces.
    for axis, colour in ((0, (235, 77, 75)), (1, (88, 196, 96)), (2, (66, 153, 240))):
        tip = [0.0, 0.0, 0.0]
        start = [0.0, 0.0, 0.0]
        sign = 1 if project([1.0 if i == axis else 0.0 for i in range(3)])[1] > 0 else -1
        start[axis] = sign * 1.0
        tip[axis] = sign * 1.95
        a, b = project(start)[0], project(tip)[0]
        draw.line((a, b), fill=colour + (255,), width=N // 22)
        r = N / 30
        draw.ellipse((b[0] - r, b[1] - r, b[0] + r, b[1] + r), fill=colour + (255,))

    image.resize((SIZE, SIZE), Image.LANCZOS).save(path)


if __name__ == '__main__':
    main(sys.argv[1])
