#!/usr/bin/env python3
"""Build a standalone two-rover recording and data-analysis dashboard.

The input is the JSONL recording written by ``live_multirobot_dashboard.py``.
The result contains its data, plots, replay controls, and evidence images in one
HTML file, so it can be opened later without ROS, a web server, or internet.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import statistics
from datetime import datetime
from pathlib import Path


ROBOT_NAMES = ("leo1", "leo2")


def number(value, default=None):
    if isinstance(value, bool):
        return float(value)
    if isinstance(value, (int, float)) and math.isfinite(value):
        return value
    return default


def nested(source, *keys, default=None):
    value = source
    for key in keys:
        if not isinstance(value, dict):
            return default
        value = value.get(key)
    return value if value is not None else default


def read_records(path):
    records = []
    rejected = 0
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            try:
                item = json.loads(line)
            except (json.JSONDecodeError, TypeError):
                rejected += 1
                continue
            if isinstance(item, dict) and isinstance(item.get("robots"), dict):
                records.append(item)
            else:
                rejected += 1
    if not records:
        raise SystemExit(f"No valid telemetry records found in {path}")
    return records, rejected


def status_and_mission(robot):
    status = robot.get("status", "unknown")
    mission = robot.get("mission") or {}
    if isinstance(status, str) and status.startswith("{"):
        try:
            decoded = json.loads(status)
        except json.JSONDecodeError:
            decoded = {}
        if isinstance(decoded, dict):
            status = decoded.get("state", "unknown")
            mission = {**decoded, **mission}
    return str(status), mission


def compact_record(record):
    result = {
        "t": number(record.get("wall_elapsed"), 0),
        "sim": number(record.get("sim_time"), 0),
        "nodes": number(record.get("node_count"), 0),
        "wall": record.get("wall_time", ""),
        "lock": bool(nested(record, "alignment", "locked", default=False)),
        "conf": number(nested(record, "alignment", "confidence")),
        "tagConf": number(nested(record, "alignment", "tag_confidence")),
        "stage": nested(record, "execution", "stage", default="UNKNOWN"),
        "stageDetail": nested(record, "execution", "detail", default=""),
        "shared": number(nested(record, "shared_map", "known_m2"), 0),
        "candidate": number(nested(record, "candidate_map", "known_m2"), 0),
        "robots": {},
    }
    transform = nested(record, "alignment", "transform")
    if isinstance(transform, dict):
        result["tf"] = {
            key: number(transform.get(key)) for key in ("x", "y", "yaw")
        }
    for name in ROBOT_NAMES:
        robot = nested(record, "robots", name, default={}) or {}
        pose = robot.get("pose") or {}
        mission_status, mission = status_and_mission(robot)
        result["robots"][name] = {
            "x": number(pose.get("x")),
            "y": number(pose.get("y")),
            "yaw": number(pose.get("yaw")),
            "v": number(nested(robot, "speed", "linear"), 0),
            "w": number(nested(robot, "speed", "angular"), 0),
            "cmdV": number(nested(robot, "command", "linear"), 0),
            "cmdW": number(nested(robot, "command", "angular"), 0),
            "scan": number(nested(robot, "scan", "hz"), 0),
            "scanAge": number(nested(robot, "scan", "age")),
            "camera": number(nested(robot, "camera", "hz"), 0),
            "cameraAge": number(nested(robot, "camera", "age")),
            "tags": number(robot.get("tags"), 0),
            "frontiers": number(robot.get("frontiers"), 0),
            "status": mission_status,
            "map": number(nested(robot, "map", "known_m2"), 0),
            "distance": number(mission.get("distance_traveled"), 0),
            "sent": number(mission.get("goals_sent"), 0),
            "success": number(mission.get("goals_succeeded"), 0),
            "failed": number(mission.get("goals_failed"), 0),
            "coverage": number(mission.get("camera_coverage"), 0),
            "recoveries": number(mission.get("watchdog_recoveries"), 0),
            "blacklist": number(mission.get("blacklist"), 0),
        }
    return result


def percentile(values, fraction):
    values = sorted(v for v in values if v is not None)
    if not values:
        return None
    position = (len(values) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return values[lower]
    return values[lower] + (values[upper] - values[lower]) * (position - lower)


def elapsed_at_first(samples, predicate):
    for sample in samples:
        if predicate(sample):
            return sample["t"]
    return None


def final_alignment_possibilities(records):
    """Latest ranked grid hypotheses, retained in the offline dashboard."""
    for record in reversed(records):
        candidates = nested(
            record, "alignment", "debug", "top_candidates", default=[])
        if not isinstance(candidates, list) or not candidates:
            continue
        result = []
        for index, candidate in enumerate(candidates, start=1):
            if not isinstance(candidate, dict):
                continue
            result.append({
                "rank": int(number(candidate.get("rank"), index)),
                "dx": number(candidate.get("dx")),
                "dy": number(candidate.get("dy")),
                "yawDeg": number(candidate.get("yaw_deg"),
                                 math.degrees(number(candidate.get("yaw"), 0))),
                "score": number(candidate.get("score"), 0),
                "relativeWeight": number(
                    candidate.get("relative_weight"), 0),
            })
        return result
    return []


def robot_summary(samples, name):
    rows = [sample["robots"][name] for sample in samples]
    final = next((row for row in reversed(rows) if row["distance"] > 0), rows[-1])
    scan = [row["scan"] for row in rows if row["scan"] > 0]
    camera = [row["camera"] for row in rows if row["camera"] > 0]
    pose_samples = sum(row["x"] is not None and row["y"] is not None for row in rows)
    scan_fresh = sum(row["scanAge"] is not None and row["scanAge"] <= 0.5 for row in rows)
    camera_fresh = sum(
        row["cameraAge"] is not None and row["cameraAge"] <= 1.0 for row in rows
    )
    moving_seconds = 0.0
    for index in range(1, len(samples)):
        delta = max(0.0, min(10.0, samples[index]["t"] - samples[index - 1]["t"]))
        if abs(rows[index]["v"]) >= 0.02 or abs(rows[index]["w"]) >= 0.05:
            moving_seconds += delta
    outcomes = final["success"] + final["failed"]
    return {
        "distance": final["distance"],
        "goalsSent": final["sent"],
        "goalsSucceeded": final["success"],
        "goalsFailed": final["failed"],
        "goalSuccessRate": (100 * final["success"] / outcomes) if outcomes else None,
        "cameraCoverage": final["coverage"],
        "recoveries": final["recoveries"],
        "blacklist": final["blacklist"],
        "finalMap": max(row["map"] for row in rows),
        "maxFrontiers": max(row["frontiers"] for row in rows),
        "maxTags": max(row["tags"] for row in rows),
        "medianScanHz": statistics.median(scan) if scan else None,
        "p05ScanHz": percentile(scan, 0.05),
        "medianCameraHz": statistics.median(camera) if camera else None,
        "poseCompleteness": 100 * pose_samples / len(rows),
        "scanFreshness": 100 * scan_fresh / len(rows),
        "cameraFreshness": 100 * camera_fresh / len(rows),
        "movingSeconds": moving_seconds,
        "maxLinearSpeed": max(abs(row["v"]) for row in rows),
        "finalStatus": rows[-1]["status"],
        "doneAt": elapsed_at_first(
            samples, lambda sample: sample["robots"][name]["status"] == "done"
        ),
    }


def event_log(samples):
    events = []
    previous_lock = False
    previous_stage = None
    previous_status = {name: None for name in ROBOT_NAMES}
    for sample in samples:
        if sample["stage"] != previous_stage and sample["stage"] != "UNKNOWN":
            events.append({
                "t": sample["t"], "kind": "execution",
                "message": (
                    f"Execution stage: {sample['stage']}"
                    + (f" — {sample['stageDetail']}" if sample["stageDetail"] else "")
                ),
            })
            previous_stage = sample["stage"]
        if sample["lock"] and not previous_lock:
            events.append({
                "t": sample["t"], "kind": "alignment",
                "message": f"Shared-map alignment locked at confidence {sample['conf'] or 0:.3f}",
            })
        previous_lock = sample["lock"]
        for name in ROBOT_NAMES:
            status = sample["robots"][name]["status"]
            if status != previous_status[name] and status not in ("waiting", "unknown"):
                events.append({
                    "t": sample["t"], "kind": name,
                    "message": f"{name} state changed to {status}",
                })
            # A restarted recorder briefly reports "waiting" until it receives
            # the latched mission status. That is a telemetry gap, not a real
            # explorer state transition.
            if status not in ("waiting", "unknown") or previous_status[name] is None:
                previous_status[name] = status
    return events


def image_uri(path):
    if not path.exists():
        return None
    suffix = path.suffix.lower()
    mime = {
        ".bmp": "image/bmp",
        ".jpg": "image/jpeg",
        ".jpeg": "image/jpeg",
        ".png": "image/png",
    }.get(suffix)
    if mime is None:
        return None
    return f"data:{mime};base64," + base64.b64encode(path.read_bytes()).decode("ascii")


def evenly_spaced(paths, limit):
    """Keep the beginning, middle, and end of a saved frame sequence."""
    paths = list(paths)
    if len(paths) <= limit:
        return paths
    if limit <= 1:
        return [paths[-1]]
    indexes = {
        round(index * (len(paths) - 1) / (limit - 1))
        for index in range(limit)
    }
    return [paths[index] for index in sorted(indexes)]


def camera_evidence(report_dir, name, per_kind=4):
    """Embed representative raw and detector-annotated views for one rover."""
    frames = []
    latest = report_dir / f"camera_{name}.jpg"
    if latest.is_file():
        frames.append({
            "label": "Latest saved dashboard view",
            "kind": "latest",
            "file": latest.name,
            "image": image_uri(latest),
        })
    frame_dir = report_dir / f"frames_{name}"
    for prefix, label in (
        ("det", "ArUco detector view"),
        ("raw", "Raw camera view"),
    ):
        selected = evenly_spaced(sorted(frame_dir.glob(f"{prefix}*.png")), per_kind)
        for path in selected:
            frames.append({
                "label": label,
                "kind": prefix,
                "file": path.name,
                "image": image_uri(path),
            })
    return [frame for frame in frames if frame["image"]]


def candidate_map_evidence(report_dir, limit=10):
    """Embed the evolving, explicitly unvetted map-merge previews."""
    frames = []
    paths = evenly_spaced(
        sorted((report_dir / "candidate_maps").glob("candidate_*.jpg")),
        limit,
    )
    for path in paths:
        time_token = next(
            (part[1:] for part in path.stem.split("_") if part.startswith("t")),
            "?",
        )
        frames.append({
            "label": f"Candidate merge at ROS t={time_token} s",
            "file": path.name,
            "image": image_uri(path),
        })
    latest = report_dir / "map_candidate.bmp"
    if latest.is_file():
        frames.append({
            "label": "Latest continuously evaluated candidate",
            "file": latest.name,
            "image": image_uri(latest),
        })
    return [frame for frame in frames if frame["image"]]


def _last_robot_value(records, name, key, predicate=lambda value: value is not None):
    for record in reversed(records):
        robot = nested(record, "robots", name, default={}) or {}
        value = robot.get(key)
        if predicate(value):
            return value
    return None


def local_map_evidence(records, report_dir, name):
    """Pair the saved occupancy map with its persistent map-frame ArUco registry."""
    map_info = _last_robot_value(
        records, name, "map",
        lambda value: isinstance(value, dict)
        and number(value.get("width"), 0) > 0
        and number(value.get("height"), 0) > 0,
    ) or {}
    markers = _last_robot_value(
        records, name, "tag_markers",
        lambda value: isinstance(value, list) and bool(value),
    ) or []
    pose = _last_robot_value(
        records, name, "map_pose", lambda value: isinstance(value, dict))
    if pose is None:
        pose = _last_robot_value(
            records, name, "pose", lambda value: isinstance(value, dict))
    goal = _last_robot_value(
        records, name, "goal", lambda value: isinstance(value, dict))
    map_image = image_uri(report_dir / f"map_{name}.bmp")
    if map_image is None:
        return None
    return {
        "image": map_image,
        "map": {
            "width": number(map_info.get("width"), 0),
            "height": number(map_info.get("height"), 0),
            "resolution": number(map_info.get("resolution"), 0),
            "origin": map_info.get("origin") or {"x": 0, "y": 0, "yaw": 0},
            "frame": map_info.get("frame", f"{name}/map"),
        },
        "markers": [
            {
                "id": marker.get("id"),
                "x": number(marker.get("x")),
                "y": number(marker.get("y")),
                "frame": marker.get("frame", f"{name}/map"),
            }
            for marker in markers if isinstance(marker, dict)
            and number(marker.get("x")) is not None
            and number(marker.get("y")) is not None
        ],
        "pose": pose,
        "goal": goal,
    }


def build_payload(records, rejected, source_path, report_dir):
    samples = [compact_record(record) for record in records]
    # wall_elapsed belongs to the recorder process and resets if that process is
    # restarted. Gazebo simulation time remains continuous across those restarts,
    # so use it as the replay axis. Records are already chronological, but sort
    # stably to make the invariant explicit.
    samples.sort(key=lambda sample: sample["sim"])
    first_sim = samples[0]["sim"]
    for sample in samples:
        sample["t"] = max(0.0, sample["sim"] - first_sim)

    # Map and mission counters arrive a message or two after a dashboard restart.
    # Preserve their last known value across only those zero-valued gaps.
    previous = {
        "shared": 0,
        "candidate": 0,
        **{
            f"{name}.{field}": 0
            for name in ROBOT_NAMES
            for field in ("map", "distance", "sent", "success", "failed")
        },
        **{f"{name}.status": None for name in ROBOT_NAMES},
    }
    for sample in samples:
        if sample["shared"] == 0 and previous["shared"] > 0:
            sample["shared"] = previous["shared"]
        elif sample["shared"] > 0:
            previous["shared"] = sample["shared"]
        if sample["candidate"] == 0 and previous["candidate"] > 0:
            sample["candidate"] = previous["candidate"]
        elif sample["candidate"] > 0:
            previous["candidate"] = sample["candidate"]
        for name in ROBOT_NAMES:
            status_key = f"{name}.status"
            status = sample["robots"][name]["status"]
            if status in ("waiting", "unknown") and previous[status_key]:
                sample["robots"][name]["status"] = previous[status_key]
            elif status not in ("waiting", "unknown"):
                previous[status_key] = status
            for field in ("map", "distance", "sent", "success", "failed"):
                key = f"{name}.{field}"
                value = sample["robots"][name][field]
                if value == 0 and previous[key] > 0:
                    sample["robots"][name][field] = previous[key]
                elif value > 0:
                    previous[key] = value

    positive_nodes = [sample["nodes"] for sample in samples if sample["nodes"] > 0]
    last_sim = samples[-1]["sim"]
    locked_at = elapsed_at_first(samples, lambda sample: sample["lock"])
    final = samples[-1]
    try:
        started_dt = datetime.fromisoformat(records[0].get("wall_time", ""))
        ended_dt = datetime.fromisoformat(records[-1].get("wall_time", ""))
        wall_duration = max(0.0, (ended_dt - started_dt).total_seconds())
    except (TypeError, ValueError):
        wall_duration = max(sample["t"] for sample in samples)
    summary = {
        "sampleCount": len(samples),
        "rejectedLines": rejected,
        "wallDuration": wall_duration,
        "simDuration": max(0, last_sim - first_sim),
        "started": records[0].get("wall_time", ""),
        "ended": records[-1].get("wall_time", ""),
        "sharedArea": max(sample["shared"] for sample in samples),
        "candidateArea": max(sample["candidate"] for sample in samples),
        "alignmentLockedAt": locked_at,
        "alignmentConfidence": final["conf"],
        "alignmentTransform": final.get("tf"),
        "alignmentPossibilities": final_alignment_possibilities(records),
        "nodeMin": min(positive_nodes) if positive_nodes else 0,
        "nodeMedian": statistics.median(positive_nodes) if positive_nodes else 0,
        "nodeMax": max(positive_nodes) if positive_nodes else 0,
        "robots": {name: robot_summary(samples, name) for name in ROBOT_NAMES},
    }
    summary["totalDistance"] = sum(
        summary["robots"][name]["distance"] for name in ROBOT_NAMES
    )
    summary["totalGoalsSucceeded"] = sum(
        summary["robots"][name]["goalsSucceeded"] for name in ROBOT_NAMES
    )
    evidence_names = {
        "mergedMap": "merged_map.png",
        "trajectories": "traj_overlay.png",
        "coverage": "coverage.png",
        "alignment": "alignment.png",
    }
    return {
        "meta": {
            "title": "Two-Rover Mission Recording",
            "source": source_path.name,
            "directory": report_dir.name,
        },
        "summary": summary,
        "events": event_log(samples),
        "samples": samples,
        "evidence": {
            key: image_uri(report_dir / filename)
            for key, filename in evidence_names.items()
        },
        "cameraEvidence": {
            name: camera_evidence(report_dir, name) for name in ROBOT_NAMES
        },
        "localMapEvidence": {
            name: local_map_evidence(records, report_dir, name)
            for name in ROBOT_NAMES
        },
        "candidateMapEvidence": candidate_map_evidence(report_dir),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report_dir", type=Path)
    parser.add_argument("--telemetry", type=Path)
    parser.add_argument("--output", default="recording_analysis.html")
    args = parser.parse_args()

    report_dir = args.report_dir.expanduser().resolve()
    telemetry = (args.telemetry or report_dir / "telemetry.jsonl").resolve()
    template = Path(__file__).with_name("recording_dashboard.html")
    if not telemetry.is_file():
        raise SystemExit(f"Telemetry file not found: {telemetry}")
    if not template.is_file():
        raise SystemExit(f"Dashboard template not found: {template}")

    records, rejected = read_records(telemetry)
    payload = build_payload(records, rejected, telemetry, report_dir)
    encoded = json.dumps(payload, separators=(",", ":")).replace("</", "<\\/")
    html = template.read_text(encoding="utf-8").replace(
        '<script id="recording-data" type="application/json">null</script>',
        f'<script id="recording-data" type="application/json">{encoded}</script>',
    )
    output = report_dir / args.output
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(html, encoding="utf-8")
    temporary.replace(output)
    print(f"Recording analysis dashboard: {output}")
    print(f"Embedded {len(records)} valid telemetry samples ({rejected} rejected lines)")


if __name__ == "__main__":
    main()
