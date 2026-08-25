"""FastDDS profile helpers for the rover firmware domain.

Call apply_udp_only_transport() before rclpy.init() in any process that
shares ROS_DOMAIN_ID with the firmware node.
"""

from __future__ import annotations

import os
from pathlib import Path


def udp_only_profile_path() -> Path:
    """Return the installed UDP-only FastDDS profile, or the source copy."""
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = (
            Path(get_package_share_directory("leo_nav2_exploration"))
            / "config"
            / "real"
            / "fastdds_udp_only.xml"
        )
        if installed.is_file():
            return installed
    except Exception:
        pass
    source = Path(__file__).resolve().parents[1] / "config" / "real" / "fastdds_udp_only.xml"
    if not source.is_file():
        raise FileNotFoundError(
            "fastdds_udp_only.xml is not installed and not next to the package"
        )
    return source


def apply_udp_only_transport() -> str:
    """Disable FastDDS shared memory for this process. Return the profile path."""
    path = str(udp_only_profile_path())
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = path
    return path
