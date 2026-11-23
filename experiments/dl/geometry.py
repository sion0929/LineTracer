"""Project world-space line, obstacle and terrain triangles into the stock camera."""

import math
import struct

import numpy as np

WIDTH, HEIGHT = 640, 480
FOCAL = WIDTH / (2 * math.tan(1.085595 / 2))
CENTER = (np.array([WIDTH, HEIGHT]) - 1) / 2
CAMERA_OFFSET = np.array([0.076, 0, 0.093])
NEAR = 0.05

_CORNERS = np.array([
    [-1, -1, -1], [-1, -1, 1], [-1, 1, -1], [-1, 1, 1],
    [1, -1, -1], [1, -1, 1], [1, 1, -1], [1, 1, 1],
])
_FACES = np.array([
    [0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1],
    [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3],
])
_TRIANGLES = _FACES[:, [[0, 1, 2], [0, 2, 3]]].reshape(-1, 3)


def rotation(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def quaternion_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def box(center, size, angles=(0, 0, 0)):
    corners = (_CORNERS * (np.asarray(size) / 2)) @ rotation(*angles).T + center
    return corners[_TRIANGLES]


def read_stl(path):
    data = path.read_bytes()
    count, = struct.unpack_from("<I", data, 80)
    triangles = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("flags", "<u2")])
    return np.frombuffer(data, triangles, count=count, offset=84)["vertices"]


def clip_near(triangle):
    polygon = []
    for previous, current in zip(np.roll(triangle, 1, axis=0), triangle):
        if (previous[0] >= NEAR) != (current[0] >= NEAR):
            fraction = (NEAR - previous[0]) / (current[0] - previous[0])
            polygon.append(previous + fraction * (current - previous))
        if current[0] >= NEAR:
            polygon.append(current)
    for i in range(1, len(polygon) - 1):
        yield np.array([polygon[0], polygon[i], polygon[i + 1]])


def rasterize(points, inverse_depth, label, mask, depth):
    x0, y0 = np.maximum(np.ceil(points.min(axis=0)), 0).astype(int)
    x1, y1 = np.minimum(np.floor(points.max(axis=0)), [WIDTH - 1, HEIGHT - 1]).astype(int)
    if x0 > x1 or y0 > y1:
        return
    a, b, c = points
    ab, ac = b - a, c - a
    determinant = ab[0] * ac[1] - ab[1] * ac[0]
    if abs(determinant) < 1e-8:
        return
    pixel_x = np.arange(x0, x1 + 1)[None, :] - a[0]
    pixel_y = np.arange(y0, y1 + 1)[:, None] - a[1]
    weight_b = (ac[1] * pixel_x - ac[0] * pixel_y) / determinant
    weight_c = (ab[0] * pixel_y - ab[1] * pixel_x) / determinant
    weight_a = 1 - weight_b - weight_c
    covered = (weight_a >= -1e-7) & (weight_b >= -1e-7) & (weight_c >= -1e-7)
    inverse = weight_a * inverse_depth[0] + weight_b * inverse_depth[1] + weight_c * inverse_depth[2]
    region = np.s_[y0:y1 + 1, x0:x1 + 1]
    closer = covered & (inverse > depth[region])
    depth[region][closer] = inverse[closer]
    mask[region][closer] = label


def render_mask(meshes, body_position, body_rotation):
    camera = body_position + body_rotation @ CAMERA_OFFSET
    mask = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    depth = np.zeros_like(mask, dtype=np.float32)
    for triangles, label in meshes:
        local = (triangles - camera) @ body_rotation
        forward = local[:, :, 0]
        for triangle in local[np.any(forward >= NEAR, axis=1)]:
            visible = (triangle,) if (triangle[:, 0] >= NEAR).all() else clip_near(triangle)
            for vertices in visible:
                inverse = 1 / vertices[:, 0]
                points = CENTER - FOCAL * vertices[:, 1:] * inverse[:, None]
                rasterize(points, inverse, label, mask, depth)
    return mask
