"""Path and parameter-file helpers used by ROS launch files."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import os
import tempfile
from typing import Any, Mapping, Sequence

import yaml


_PROFILE_DIRECTORIES = {
    "sim_leo1": "sim",
    "real_root": "real",
}


@dataclass(frozen=True)
class ProfilePaths:
    package_share: Path
    profile_name: str
    profile_directory: Path
    nav2: Path
    scan_filter: Path
    slam: Path
    collision_monitor: Path
    velocity_guard: Path
    frontier: Path
    behavior_tree: Path


def resolve_profile_paths(package_share: os.PathLike[str] | str, profile: str) -> ProfilePaths:
    try:
        directory_name = _PROFILE_DIRECTORIES[profile]
    except KeyError as exc:
        choices = ", ".join(sorted(_PROFILE_DIRECTORIES))
        raise ValueError(f"unknown profile {profile!r}; expected one of: {choices}") from exc

    share = Path(package_share).resolve()
    directory = share / "config" / directory_name
    paths = ProfilePaths(
        package_share=share,
        profile_name=profile,
        profile_directory=directory,
        nav2=directory / "nav2.yaml",
        scan_filter=directory / "scan_filter.yaml",
        slam=directory / "slam.yaml",
        collision_monitor=directory / "collision_monitor.yaml",
        velocity_guard=directory / "velocity_guard.yaml",
        frontier=directory / "frontier.yaml",
        behavior_tree=share / "behavior_trees" / "navigate_to_pose_doorway_recovery.xml",
    )
    missing = [
        path
        for path in (
            paths.nav2,
            paths.scan_filter,
            paths.slam,
            paths.collision_monitor,
            paths.velocity_guard,
            paths.frontier,
            paths.behavior_tree,
        )
        if not path.is_file()
    ]
    if missing:
        joined = ", ".join(str(path) for path in missing)
        raise FileNotFoundError(f"profile {profile!r} is incomplete: {joined}")
    return paths


def sample_lattice_path(nav2_smac_planner_share: os.PathLike[str] | str) -> Path:
    path = (
        Path(nav2_smac_planner_share).resolve()
        / "sample_primitives"
        / "5cm_resolution"
        / "0.5m_turning_radius"
        / "diff"
        / "output.json"
    )
    if not path.is_file():
        raise FileNotFoundError(
            "Nav2's installed differential-drive State Lattice sample was not found at "
            f"{path}. Install ros-humble-nav2-smac-planner."
        )
    return path


def _replace_exact(value: Any, replacements: Mapping[str, str]) -> Any:
    if isinstance(value, dict):
        return {key: _replace_exact(item, replacements) for key, item in value.items()}
    if isinstance(value, list):
        return [_replace_exact(item, replacements) for item in value]
    if isinstance(value, str) and value in replacements:
        return replacements[value]
    return value


def materialize_parameter_file(
    source: os.PathLike[str] | str,
    replacements: Mapping[str, str],
    *,
    output_directory: os.PathLike[str] | str | None = None,
    path_overrides: Mapping[Sequence[str], Any] | None = None,
) -> Path:
    """Copy a YAML parameter file while replacing exact placeholder values."""

    source_path = Path(source).resolve()
    data = yaml.safe_load(source_path.read_text(encoding="utf-8"))
    rewritten = _replace_exact(data, replacements)
    for path, override in (path_overrides or {}).items():
        if not path:
            raise ValueError("override path must not be empty")
        cursor = rewritten
        for key in path[:-1]:
            if not isinstance(cursor, dict) or key not in cursor:
                raise KeyError(f"parameter override path does not exist: {tuple(path)!r}")
            cursor = cursor[key]
        if not isinstance(cursor, dict) or path[-1] not in cursor:
            raise KeyError(f"parameter override path does not exist: {tuple(path)!r}")
        cursor[path[-1]] = override
    directory = Path(output_directory).resolve() if output_directory else Path(tempfile.gettempdir())
    directory.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(
        prefix=f"leo_nav2_{source_path.stem}_",
        suffix=".yaml",
        dir=directory,
        text=True,
    )
    output = Path(name)
    with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
        yaml.safe_dump(rewritten, handle, sort_keys=False, width=140)
    return output
