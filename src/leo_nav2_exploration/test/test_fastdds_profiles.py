"""FastDDS UDP-only profile used on the rover firmware domain."""

import os
from pathlib import Path

from leo_nav2_exploration.fastdds_profiles import apply_udp_only_transport, udp_only_profile_path


def test_udp_only_profile_disables_builtin_shared_memory():
    xml = udp_only_profile_path().read_text(encoding="utf-8")
    assert "<type>UDPv4</type>" in xml
    assert "<useBuiltinTransports>false</useBuiltinTransports>" in xml
    assert "<transport_id>udp_transport</transport_id>" in xml


def test_apply_udp_only_transport_sets_env_before_rclpy(monkeypatch):
    monkeypatch.delenv("FASTRTPS_DEFAULT_PROFILES_FILE", raising=False)
    path = apply_udp_only_transport()
    assert Path(path).is_file()
    assert path.endswith("fastdds_udp_only.xml")
    assert os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] == path
