"""Per-rover shared mapping: each rover merges the peer's map LOCALLY.

The central stack (shared_align.launch.py) runs one aligner, one bridge and
one merger for the whole fleet -- a single point of failure that in the lab
would live on the laptop. Here every rover gets its own aligner + merger
pair, subscribed to the peer's /map, publishing /{self}/shared_map **in the
rover's own map frame**:

* No node that both rovers depend on: kill either half (or the laptop) and
  the other rover keeps aligning, merging and exploring on what it has.
* The merged map being in the rover's own frame means the frontier mask
  needs no alignment TF at all -- the explorer sees frame_id == its own map
  frame and uses the identity offset.
* Only /leo{i}/map crosses the network (plus tag detections when markers are
  on). Scans, clouds and costmaps stay home (commit d241087: our own DDS
  traffic starved the rover firmware).

Alignment is markerfree by default: the benchmarked global matcher with
margin abstention (zero confident-wrong on the 10 recorded pairs). Each
rover's merger consumes its OWN aligner's accepted transform -- which the
aligner only publishes after the polished-hit floor, the ambiguity margin
and the jump checks -- so the vetting the central bridge used to do is
already inside the per-rover decision.

The optional coordination TF (leo1/map -> leo2/map for peer-position
lookups) stays with the central bridge when one is running; nothing here
depends on it.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rover_pair(ns, peer, use_sim_time, mode, min_conf):
    aligner = Node(
        package="multi_robot_shared_mapping",
        executable="map_based_aligner",
        name="peer_aligner",
        namespace=ns,
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "alignment_mode": mode,
            # map1 = SELF: the recovered transform maps the peer's grid into
            # this rover's own frame, which is the frame it navigates in.
            "map1_topic": f"/{ns}/map",
            "map2_topic": f"/{peer}/map",
            "output_topic": f"/{ns}/peer_transform",
            "candidate_topic": f"/{ns}/alignment_candidate",
            "confidence_topic": f"/{ns}/alignment_confidence",
            "debug_topic": f"/{ns}/alignment_debug_json",
            "recovery_topic": f"/{ns}/alignment_recovery",
            "debug_map_topic": f"/{ns}/peer_map_transformed_debug",
            "parent_map_frame": f"{ns}/map",
            "child_map_frame": f"{peer}/map_estimated_by_{ns}",
            "min_alignment_confidence": min_conf,
        }],
    )
    merger = Node(
        package="multi_robot_shared_mapping",
        executable="shared_map_merger",
        name="shared_map_merger",
        namespace=ns,
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "alignment_mode": mode,
            "map1_topic": f"/{ns}/map",
            "map2_topic": f"/{peer}/map",
            "shared_map_topic": f"/{ns}/shared_map",
            "shared_map_raw_topic": f"/{ns}/shared_map_raw",
            "shared_map_cleaned_topic": f"/{ns}/shared_map_cleaned",
            "shared_map_candidate_topic": f"/{ns}/shared_map_candidate",
            "shared_frame_id": f"{ns}/map",
            "map_transform_topic": f"/{ns}/peer_transform",
            "candidate_transform_topic": f"/{ns}/alignment_candidate",
            "confidence_topic": f"/{ns}/alignment_confidence",
            "min_alignment_confidence": min_conf,
        }],
    )
    return [aligner, merger]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    mode = LaunchConfiguration("alignment_mode")
    min_conf = LaunchConfiguration("min_alignment_confidence")

    nodes = []
    nodes += _rover_pair("leo1", "leo2", use_sim_time, mode, min_conf)
    nodes += _rover_pair("leo2", "leo1", use_sim_time, mode, min_conf)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("alignment_mode", default_value="markerfree"),
        DeclareLaunchArgument("min_alignment_confidence", default_value="0.45"),
    ] + nodes)
