"""Latest-only queue helper for the firmware domain bridge."""

from queue import Full, Queue

from leo_nav2_exploration.firmware_bridge import put_latest


def test_put_latest_keeps_only_the_newest_sample():
    queue = Queue(maxsize=1)
    put_latest(queue, "old")
    put_latest(queue, "new")
    assert queue.get_nowait() == "new"
    assert queue.empty()


def test_put_latest_does_not_raise_when_consumer_is_absent():
    queue = Queue(maxsize=1)
    put_latest(queue, 1)
    put_latest(queue, 2)
    put_latest(queue, 3)
    assert queue.get_nowait() == 3


def test_full_queue_without_helper_still_raises():
    queue = Queue(maxsize=1)
    queue.put_nowait("x")
    try:
        queue.put_nowait("y")
        raise AssertionError("expected Full")
    except Full:
        pass


def test_defaults_target_jetson02_root_topics_not_rover4(monkeypatch):
    monkeypatch.delenv("LEO_FIRMWARE_DOMAIN", raising=False)
    monkeypatch.delenv("LEO_FW_ODOM_TOPIC", raising=False)
    monkeypatch.delenv("LEO_FW_BATTERY_TOPIC", raising=False)
    monkeypatch.delenv("LEO_FW_CMD_VEL_TOPIC", raising=False)
    from leo_nav2_exploration.firmware_bridge import firmware_bridge_settings
    settings = firmware_bridge_settings()
    assert settings["firmware_domain"] == "2"
    assert settings["odom_topic"] == "/wheel_odom"
    assert settings["battery_topic"] == "/firmware/battery_averaged"
    assert settings["cmd_topic"] == "/cmd_vel"
    assert "/rob_2" not in settings["cmd_topic"]


def test_rover4_overrides_do_not_change_defaults(monkeypatch):
    monkeypatch.setenv("LEO_FIRMWARE_DOMAIN", "4")
    monkeypatch.setenv("LEO_FW_ODOM_TOPIC", "/rob_2/firmware/wheel_odom")
    monkeypatch.setenv("LEO_FW_CMD_VEL_TOPIC", "/rob_2/cmd_vel")
    from leo_nav2_exploration.firmware_bridge import firmware_bridge_settings
    settings = firmware_bridge_settings()
    assert settings["firmware_domain"] == "4"
    assert settings["cmd_topic"] == "/rob_2/cmd_vel"
