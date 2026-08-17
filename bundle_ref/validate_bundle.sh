#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE="$ROOT/src/leo_nav2_exploration"
MANIFEST="$ROOT/MANIFEST.sha256"

fail() {
  printf 'VALIDATION FAILED: %s\n' "$*" >&2
  exit 1
}

for command in python bash sha256sum; do
  command -v "$command" >/dev/null 2>&1 || fail "required command '$command' is missing"
done
[[ -f "$PACKAGE/package.xml" ]] || fail "package.xml is missing"
[[ -d "$PACKAGE/test" ]] || fail "test directory is missing"

printf '1/7 Python unit and contract tests\n'
(
  cd "$PACKAGE"
  PYTHONPATH="$PACKAGE${PYTHONPATH:+:$PYTHONPATH}" python -m pytest -q test
)

printf '2/7 Python source compilation\n'
python -m compileall -q -f \
  "$PACKAGE/leo_nav2_exploration" \
  "$PACKAGE/launch"

printf '3/7 YAML, XML, SDF, and repository-file parsing\n'
ROOT="$ROOT" python - <<'PY'
import os
from pathlib import Path
import xml.etree.ElementTree as ElementTree

import yaml

root = Path(os.environ["ROOT"])
yaml_files = sorted(root.rglob("*.yaml")) + sorted(root.rglob("*.yml")) + [root / "dependencies.repos"]
for path in dict.fromkeys(yaml_files):
    with path.open("r", encoding="utf-8") as handle:
        yaml.safe_load(handle)
xml_files = []
for pattern in ("*.xml", "*.sdf"):
    xml_files.extend(root.rglob(pattern))
xml_files.extend(root.rglob("model.config"))
for path in sorted(set(xml_files)):
    ElementTree.parse(path)
print(f"parsed {len(dict.fromkeys(yaml_files))} YAML files and {len(set(xml_files))} XML/SDF files")
PY

printf '4/7 Shell syntax and executable permissions\n'
while IFS= read -r -d '' script; do
  bash -n "$script"
  [[ -x "$script" ]] || fail "script is not executable: ${script#$ROOT/}"
done < <(find "$ROOT/scripts" -maxdepth 1 -type f -name '*.sh' -print0)
bash -n "$ROOT/validate_bundle.sh"
[[ -x "$ROOT/validate_bundle.sh" ]] || fail "validate_bundle.sh is not executable"

printf '5/7 Standalone-overlay ownership and command-chain checks\n'
LAUNCH="$PACKAGE/launch/navigation_overlay.launch.py"
! grep -q "static_transform_publisher" "$LAUNCH" || fail "overlay must not publish static TF"
! grep -q "robot_state_publisher" "$LAUNCH" || fail "overlay must not publish robot state TF"
! grep -q "wheel_odom_tf" "$LAUNCH" || fail "overlay must not publish odometry TF"
grep -q "cmd_vel_nav" "$LAUNCH" || fail "Nav2 command stage is missing"
grep -q "cmd_vel_smoothed" "$LAUNCH" || fail "velocity smoother stage is missing"
grep -q "cmd_vel_guarded" "$PACKAGE/config/real/collision_monitor.yaml" || fail "guarded command stage is missing"
grep -q "cmd_vel_out_topic: /cmd_vel" "$PACKAGE/config/real/collision_monitor.yaml" || fail "real final command is not owned by Collision Monitor"
grep -q "cmd_vel_out_topic: /leo1/cmd_vel" "$PACKAGE/config/sim/collision_monitor.yaml" || fail "sim final command is not owned by Collision Monitor"
grep -q "scan_to_scan_filter_chain" "$LAUNCH" || fail "LaserScan self filter is missing"

printf '6/7 Package and archive-content checks\n'
ROOT="$ROOT" python - <<'PY'
import os
from pathlib import Path
import xml.etree.ElementTree as ElementTree

root = Path(os.environ["ROOT"])
package = root / "src" / "leo_nav2_exploration"
xml_root = ElementTree.parse(package / "package.xml").getroot()
name = xml_root.findtext("name")
if name != "leo_nav2_exploration":
    raise SystemExit(f"unexpected package name: {name!r}")
required = {
    "laser_filters",
    "nav2_smac_planner",
    "nav2_rotation_shim_controller",
    "nav2_collision_monitor",
    "slam_toolbox",
    "frontier_exploration_ros2",
}
deps = {node.text.strip() for node in xml_root.findall("exec_depend") if node.text}
missing = sorted(required - deps)
if missing:
    raise SystemExit(f"missing runtime dependencies: {missing}")
required_docs = {
    "README.md",
    "INTEGRATION_FOR_CLAUDE_CODE.md",
    "CALIBRATION.md",
    "SIMULATION_TEST_PLAN.md",
}
actual_docs = {path.name for path in (root / "docs").glob("*.md")}
if not required_docs <= actual_docs:
    raise SystemExit(f"missing documents: {sorted(required_docs - actual_docs)}")
for profile in ("sim", "real"):
    config = package / "config" / profile
    for name in ("nav2", "slam", "scan_filter", "collision_monitor", "velocity_guard", "frontier"):
        if not (config / f"{name}.yaml").is_file():
            raise SystemExit(f"missing {profile}/{name}.yaml")
print("package metadata and required bundle content are present")
PY

printf '7/7 SHA-256 integrity manifest\n'
if [[ -s "$MANIFEST" ]]; then
  (cd "$ROOT" && sha256sum -c MANIFEST.sha256)
else
  printf 'MANIFEST.sha256 not generated yet; integrity check skipped for staging validation.\n'
fi

if [[ -r /opt/ros/humble/setup.bash ]]; then
  printf 'ROS 2 Humble detected. Source/build/runtime plugin checks should be run in the target workspace.\n'
else
  printf 'ROS 2 Humble is not installed in this validator environment; runtime lifecycle/Gazebo checks remain external.\n'
fi
printf 'Bundle validation passed.\n'
