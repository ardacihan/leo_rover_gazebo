"""Alignment + shared-map merging only -- no Gazebo, no SLAM, no robots.

`shared_mapping_demo.launch.py` brings up its own simulator and its own pair of
slam_toolbox nodes, which makes it a self-contained demo and unusable as a
component. The integrated run needs the same six nodes layered on top of the
sim that `two_robots_gpu.launch.py` + `slam_multi.launch.py` already started,
so this file is the demo's second half with the world removed.

Defaults differ from the demo's in the three ways Phase 1 requires:

* `alignment_mode` is **hybrid**, not `fixed`. `fixed` publishes a static
  `leo1/map -> leo2/map` built from the true spawn offset -- it hands the
  merger the answer. `hybrid` recovers it from tags and cross-checks it by
  grid matching.
* `enable_tag_alignment` and `enable_alignment_evaluation` are **true**, so
  the recovered transform is *measured* against ground truth rather than
  assumed to be it. The ground truth is used for scoring only; no node
  consumes it for alignment.
* `apriltag_detection_node` is **off**. The tags this run uses are ArUco,
  detected by `leo_nav2_exploration/aruco_detector` (the hardware-validated
  one) publishing the same `MarkerArray` contract on `/leo{i}/tag_detections`.

`min_tags` stays at **2**. Two common landmarks are the minimum for a full 2D
transform; one gives a bearing hint whose yaw is only as good as a single
marker's normal. Rather than lower the bar, the markers were placed so that
both rovers cross a shared area -- depot ids 3/5/6 in the south-central
corridor, office ids 1/2/8 along the corridor every room opens onto.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = LaunchConfiguration

    use_sim_time = cfg("use_sim_time")
    alignment_mode = cfg("alignment_mode")
    min_conf = cfg("min_alignment_confidence")
    gt_x, gt_y, gt_yaw = cfg("ground_truth_x"), cfg("ground_truth_y"), cfg("ground_truth_yaw")

    shared_map_merger = Node(
        package="multi_robot_shared_mapping",
        executable="shared_map_merger",
        name="shared_map_merger",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "map1_topic": "/leo1/map",
            "map2_topic": "/leo2/map",
            "shared_map_topic": "/shared_map",
            "shared_frame_id": "leo1/map",
            "alignment_mode": alignment_mode,
            "estimated_transform_topic": "/estimated_transform/leo2_to_leo1",
            # The VETTED transform, not the raw grid estimate: the bridge
            # applies the tag-evidence and tag-vs-grid agreement checks, and the
            # merger must fuse under exactly the transform the rovers are
            # coordinating in. Subscribing to /map_based_transform here is how
            # depot_world got a doubled world out of a 180-degree-flipped grid
            # match that the bridge had already rejected.
            "map_transform_topic": "/vetted_transform/leo2_to_leo1",
            "confidence_topic": "/vetted_alignment_confidence",
            "min_alignment_confidence": min_conf,
        }],
    )

    tag_based_map_aligner = Node(
        package="multi_robot_shared_mapping",
        executable="tag_based_map_aligner",
        name="tag_based_map_aligner",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "leo1_detections_topic": "/leo1/tag_detections",
            "leo2_detections_topic": "/leo2/tag_detections",
            "landmark_persistence": True,
            "min_tags": cfg("min_tags"),
            # Scoring only -- never fed back into the estimate.
            "ground_truth_x": gt_x,
            "ground_truth_y": gt_y,
            "ground_truth_yaw": gt_yaw,
            "compare_to_ground_truth": cfg("compare_to_ground_truth"),
        }],
        condition=IfCondition(cfg("enable_tag_alignment")),
    )

    map_based_aligner = Node(
        package="multi_robot_shared_mapping",
        executable="map_based_aligner",
        name="map_based_aligner",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "alignment_mode": alignment_mode,
            "min_alignment_confidence": min_conf,
        }],
        condition=IfCondition(PythonExpression([
            "'", cfg("enable_tag_alignment"), "' == 'true' or '",
            cfg("enable_map_alignment"), "' == 'true'",
        ])),
    )

    # The piece that makes coordination possible at all: see the node's
    # docstring. Nothing is broadcast until the estimate is trusted.
    alignment_tf_bridge = Node(
        package="multi_robot_shared_mapping",
        executable="alignment_tf_bridge",
        name="alignment_tf_bridge",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "parent_frame": "leo1/map",
            "child_frame": "leo2/map",
            "min_confidence": min_conf,
            # require_tag_evidence was the defense against the OLD global
            # grid matcher, which locked confident 180-degree flips. In
            # markerfree mode the aligner's own margin abstention is that
            # defense (benchmarked: zero confident-wrong on 10 pairs), and
            # there are no tags by construction -- keeping the tag gate on
            # would make marker-free locking impossible.
            "require_tag_evidence": ParameterValue(
                PythonExpression(["'", alignment_mode, "' != 'markerfree'"]),
                value_type=bool),
        }],
        condition=IfCondition(cfg("enable_alignment_tf")),
    )

    robot_state_registry = Node(
        package="multi_robot_shared_mapping",
        executable="robot_state_registry",
        name="robot_state_registry",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # fixed | tag | map | hybrid. Never 'fixed' for an honest run.
        DeclareLaunchArgument("alignment_mode", default_value="hybrid"),
        DeclareLaunchArgument("enable_tag_alignment", default_value="true"),
        DeclareLaunchArgument("enable_map_alignment", default_value="true"),
        DeclareLaunchArgument("enable_alignment_tf", default_value="true"),
        DeclareLaunchArgument("min_alignment_confidence", default_value="0.5"),
        DeclareLaunchArgument("min_tags", default_value="2"),
        # True spawn offset of leo2 relative to leo1, for scoring only.
        DeclareLaunchArgument("compare_to_ground_truth", default_value="true"),
        DeclareLaunchArgument("ground_truth_x", default_value="0.0"),
        DeclareLaunchArgument("ground_truth_y", default_value="0.0"),
        DeclareLaunchArgument("ground_truth_yaw", default_value="0.0"),

        shared_map_merger,
        tag_based_map_aligner,
        map_based_aligner,
        alignment_tf_bridge,
        robot_state_registry,
    ])
