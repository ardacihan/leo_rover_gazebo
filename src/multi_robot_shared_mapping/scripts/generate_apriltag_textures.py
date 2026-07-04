#!/usr/bin/env python3
"""Generate tag36h11 PNG textures for Gazebo AprilTag models."""

import os
import sys

import cv2
import numpy as np

TAG_IDS = (0, 1, 2, 3, 4, 5)
TAG_SIZE_PX = 400


def generate_tag_image(tag_id: int) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    image = np.zeros((TAG_SIZE_PX, TAG_SIZE_PX), dtype=np.uint8)
    cv2.aruco.drawMarker(dictionary, tag_id, TAG_SIZE_PX, image, 1)
    return image


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_root = os.path.dirname(script_dir)
    assets_dir = os.path.join(pkg_root, "assets", "apriltags")

    os.makedirs(assets_dir, exist_ok=True)

    for tag_id in TAG_IDS:
        png_name = f"tag36h11_{tag_id}.png"
        assets_path = os.path.join(assets_dir, png_name)
        cv2.imwrite(assets_path, generate_tag_image(tag_id))
        print(f"Wrote {assets_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
