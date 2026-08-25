"""Camera-based collision avoidance is a second Collision Monitor source.

Two failure modes are worth pinning. First, Collision Monitor rejects a
configured observation source whose topic never publishes, so the source list
and the depth pipeline must be enabled together -- listing depth_scan while
the converter is off would leave the monitor permanently degraded. Second, the
RealSense publishes its own frames rooted at camera_link with no link to the
robot, so the static transform must exist or every depth point is untransformable.
"""

import ast
import pathlib
import unittest

import yaml


PACKAGE = pathlib.Path(__file__).parents[1]
LAUNCH = PACKAGE / "launch" / "safe_mapping.launch.py"
LIDAR_ONLY = PACKAGE / "config" / "collision_monitor_params.yaml"
WITH_CAMERA = PACKAGE / "config" / "collision_monitor_camera_params.yaml"


def _load(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))[
        "collision_monitor"
    ]["ros__parameters"]


class CollisionConfigTests(unittest.TestCase):
    def test_lidar_only_config_lists_only_the_lidar(self):
        params = _load(LIDAR_ONLY)
        self.assertEqual(params["observation_sources"], ["scan"])

    def test_camera_config_adds_depth_without_removing_the_lidar(self):
        """The camera supplements the LIDAR; it must never replace it --
        the D456 is blind closer than ~0.45 m and covers only ~87 degrees."""
        params = _load(WITH_CAMERA)
        self.assertEqual(params["observation_sources"], ["scan", "depth_scan"])
        self.assertTrue(params["scan"]["enabled"])
        self.assertTrue(params["depth_scan"]["enabled"])
        self.assertEqual(params["depth_scan"]["type"], "scan")

    def test_both_configs_agree_on_the_safety_envelope(self):
        """Enabling the camera must not silently change the footprint,
        speeds, or timeouts."""
        lidar, camera = _load(LIDAR_ONLY), _load(WITH_CAMERA)
        for key in (
            "base_frame_id", "odom_frame_id", "cmd_vel_in_topic",
            "cmd_vel_out_topic", "source_timeout", "transform_tolerance",
            "base_shift_correction", "polygons",
        ):
            self.assertEqual(lidar[key], camera[key], key)
        self.assertEqual(
            lidar["FootprintApproach"], camera["FootprintApproach"]
        )


class LaunchWiringTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = LAUNCH.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source)

    def _nodes_named(self, name):
        found = []
        for call in ast.walk(self.tree):
            if not isinstance(call, ast.Call):
                continue
            if not isinstance(call.func, ast.Name) or call.func.id != "Node":
                continue
            keywords = {k.arg: k.value for k in call.keywords}
            node_name = keywords.get("name")
            if isinstance(node_name, ast.Constant) and node_name.value == name:
                found.append(keywords)
        return found

    def test_camera_transform_and_converter_share_one_condition(self):
        """Both must be gated on use_camera_collision, so the source list and
        the pipeline can never disagree."""
        for name in ("camera_static_transform", "depth_obstacle_scan"):
            nodes = self._nodes_named(name)
            self.assertEqual(len(nodes), 1, name)
            condition = nodes[0].get("condition")
            self.assertIsNotNone(condition, f"{name} must be conditional")
            self.assertIn("use_camera", ast.unparse(condition))

    def test_static_transform_parents_camera_link_to_base_link(self):
        node = self._nodes_named("camera_static_transform")[0]
        arguments = ast.unparse(node["arguments"])
        self.assertIn("base_link", arguments)
        self.assertIn("camera_link", arguments)

    def _argument_default(self, wanted):
        for call in ast.walk(self.tree):
            if not isinstance(call, ast.Call):
                continue
            if not isinstance(call.func, ast.Name):
                continue
            if call.func.id != "DeclareLaunchArgument" or not call.args:
                continue
            name = call.args[0]
            if isinstance(name, ast.Constant) and name.value == wanted:
                return [
                    k.value.value for k in call.keywords
                    if k.arg == "default_value"
                ][0]
        self.fail(f"{wanted} argument not declared")

    def test_depth_range_min_respects_the_sensor_blind_zone(self):
        """A range_min below the D456's ~0.45 m minimum would invent
        obstacles out of invalid depth pixels."""
        self.assertGreaterEqual(
            float(self._argument_default("depth_range_min")), 0.45
        )

    def test_obstacle_height_band_excludes_the_floor_and_clears_the_rover(self):
        """The lower bound is the floor rejection that motivated this node;
        the upper bound must stay above the rover so it cannot see 'through'
        a real obstacle, but below ceiling height."""
        low = float(self._argument_default("min_obstacle_height"))
        high = float(self._argument_default("max_obstacle_height"))
        self.assertGreater(low, 0.0, "0.0 would put the floor back in")
        self.assertLessEqual(low, 0.10, "too high would miss low boxes")
        self.assertGreaterEqual(high, 0.40)
        self.assertLess(high, 1.0)

    def test_depth_scan_is_decimated_for_cpu_headroom(self):
        """Collision Monitor drops sources it cannot keep up with; this node
        must not consume the CPU that costs."""
        self.assertGreaterEqual(
            int(self._argument_default("depth_pixel_step")), 2
        )


if __name__ == "__main__":
    unittest.main()
