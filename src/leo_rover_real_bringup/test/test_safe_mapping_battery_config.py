import ast
import pathlib
import unittest


PACKAGE = pathlib.Path(__file__).parents[1]
LAUNCH_FILE = PACKAGE / "launch" / "safe_mapping.launch.py"
BATTERY_NODES = {"safety_command_gate.py", "safe_room_explorer.py"}
BATTERY_NODE_FILES = [PACKAGE / "scripts" / name for name in BATTERY_NODES]


def _constant(node):
    return node.value if isinstance(node, ast.Constant) else None


def _declared_arguments(tree):
    """Map every DeclareLaunchArgument name to its literal default."""
    arguments = {}
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        if not isinstance(call.func, ast.Name):
            continue
        if call.func.id != "DeclareLaunchArgument" or not call.args:
            continue
        name = _constant(call.args[0])
        defaults = [
            _constant(keyword.value)
            for keyword in call.keywords
            if keyword.arg == "default_value"
        ]
        arguments[name] = defaults[0] if defaults else None
    return arguments


class SafeMappingBatteryConfigTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))

    def test_battery_launch_arguments_exist_with_safe_defaults(self):
        arguments = _declared_arguments(self.tree)

        self.assertEqual(arguments["minimum_battery_voltage"], "5.0")
        self.assertIn("battery_topic", arguments)

    def test_rover_specific_toggles_default_to_the_rover_1_topology(self):
        arguments = _declared_arguments(self.tree)

        # Rover 1 owns none of these, so the package still starts them by
        # default.  Rover 4 already publishes both transforms and runs its own
        # mapper, and must switch them off.
        for name in ("start_lidar_tf", "start_wheel_odom_tf", "start_slam"):
            self.assertEqual(arguments[name], "true", name)
        self.assertEqual(arguments["odom_topic"], "/wheel_odom_integrated")

    def test_explorer_odom_topic_is_configurable(self):
        # Hardcoding this stranded the explorer on any rover that does not run
        # wheel_odom_tf.py, because nothing publishes /wheel_odom_integrated.
        for call in ast.walk(self.tree):
            if not isinstance(call, ast.Call):
                continue
            if not isinstance(call.func, ast.Name) or call.func.id != "Node":
                continue

            keywords = {keyword.arg: keyword.value for keyword in call.keywords}
            if _constant(keywords.get("executable")) != "safe_room_explorer.py":
                continue

            for node in ast.walk(keywords["parameters"]):
                if not isinstance(node, ast.Dict):
                    continue
                for key, value in zip(node.keys, node.values):
                    if _constant(key) == "odom_topic":
                        self.assertIsInstance(value, ast.Name)
                        self.assertEqual(value.id, "odom_topic")
                        return

        self.fail("explorer odom_topic parameter not found")

    def test_battery_node_defaults_are_5_volts(self):
        for path in BATTERY_NODE_FILES:
            tree = ast.parse(path.read_text(encoding="utf-8"))
            defaults = []
            for call in ast.walk(tree):
                if not isinstance(call, ast.Call) or len(call.args) < 2:
                    continue
                if not isinstance(call.func, ast.Attribute):
                    continue
                if call.func.attr != "declare_parameter":
                    continue
                if _constant(call.args[0]) == "minimum_battery_voltage":
                    defaults.append(_constant(call.args[1]))

            self.assertEqual(defaults, [5.0], path.name)

    def test_battery_arguments_are_propagated_to_all_battery_nodes(self):
        configured_nodes = set()
        for call in ast.walk(self.tree):
            if not isinstance(call, ast.Call):
                continue
            if not isinstance(call.func, ast.Name) or call.func.id != "Node":
                continue

            keywords = {keyword.arg: keyword.value for keyword in call.keywords}
            executable = _constant(keywords.get("executable"))
            if executable not in BATTERY_NODES:
                continue

            parameter_names = {}
            for node in ast.walk(keywords["parameters"]):
                if not isinstance(node, ast.Dict):
                    continue
                for key, value in zip(node.keys, node.values):
                    parameter_names[_constant(key)] = (
                        value.id if isinstance(value, ast.Name) else None
                    )

            self.assertEqual(parameter_names["battery_topic"], "battery_topic")
            self.assertEqual(
                parameter_names["minimum_battery_voltage"],
                "minimum_battery_voltage",
            )
            # Both nodes derive front/rear sectors from raw scan angles, so
            # both must receive the mount offset or they disagree about which
            # way the rover faces.
            self.assertEqual(
                parameter_names["scan_yaw_offset"], "scan_yaw_offset"
            )
            configured_nodes.add(executable)

        self.assertEqual(configured_nodes, BATTERY_NODES)


if __name__ == "__main__":
    unittest.main()
