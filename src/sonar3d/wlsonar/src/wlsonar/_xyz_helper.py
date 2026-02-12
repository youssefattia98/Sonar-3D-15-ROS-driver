"""Helpers for RangeImage."""

import math
from typing import List, Tuple

from .range_image_protocol import RangeImage


def range_image_to_xyz(range_image: RangeImage) -> List[Tuple[float, float, float]]:
    """range_image_to_xyz returns voxels of RangeImage as x,y,z in meters.

    Args:
        range_image: the RangeImage

    Returns:
        List of (x, y, z) tuples in meters
    """
    voxels = []

    max_pixel_x = range_image.width - 1
    max_pixel_y = range_image.height - 1

    fov_h = math.radians(range_image.fov_horizontal)
    fov_v = math.radians(range_image.fov_vertical)

    for pixel_x in range(range_image.width):
        for pixel_y in range(range_image.height):
            pixel_idx = pixel_y * range_image.width + pixel_x
            pixel_value = range_image.image_pixel_data[pixel_idx]
            if pixel_value == 0:
                # No data for this pixel
                continue

            distance_meters = pixel_value * range_image.image_pixel_scale
            yaw_rad = (pixel_x / max_pixel_x) * fov_h - fov_h / 2
            pitch_rad = (pixel_y / max_pixel_y) * fov_v - fov_v / 2

            x = distance_meters * math.cos(pitch_rad) * math.cos(yaw_rad)
            y = distance_meters * math.cos(pitch_rad) * math.sin(yaw_rad)
            z = -distance_meters * math.sin(pitch_rad)
            voxels.append((x, y, z))

    return voxels
