from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parents[1]


def test_merged_landmarks_and_occupancy_use_the_same_vetted_transform():
    launch = (PACKAGE / "launch" / "shared_align.launch.py").read_text(
        encoding="utf-8")
    assert '"accepted_transform_topic": "/vetted_transform/leo2_to_leo1"' in launch
    assert '"accepted_confidence_topic": "/vetted_alignment_confidence"' in launch
    assert '"map_transform_topic": "/vetted_transform/leo2_to_leo1"' in launch
    assert '"confidence_topic": "/vetted_alignment_confidence"' in launch


def test_alignment_confidences_are_paired_with_their_accepted_transforms():
    launch = (PACKAGE / "launch" / "shared_align.launch.py").read_text(
        encoding="utf-8")
    map_source = (PACKAGE / "multi_robot_shared_mapping" /
                  "map_based_aligner.py").read_text(encoding="utf-8")
    tag_source = (PACKAGE / "multi_robot_shared_mapping" /
                  "tag_based_map_aligner.py").read_text(encoding="utf-8")
    assert '"confidence_topic": "/map_based_accepted_confidence"' in launch
    assert '"leo1_detections_topic": "/leo1/aruco_markers"' in launch
    assert 'self.accepted_confidence_pub.publish' in map_source
    assert 'self.last_transform_confidence' in tag_source


def test_relative_estimation_start_is_logged_and_published():
    source = (PACKAGE / "multi_robot_shared_mapping" /
              "map_based_aligner.py").read_text(encoding="utf-8")
    assert "RELATIVE POSITION ESTIMATION STARTED" in source
    assert "relative_estimation_started_at" in source


def test_merged_marker_conversion_is_explicitly_logged():
    source = (PACKAGE / "multi_robot_shared_mapping" /
              "tag_based_map_aligner.py").read_text(encoding="utf-8")
    assert "MERGED MARKER CONVERSION ENABLED" in source
    assert "vetted transform used by shared_map_merger" in source


def test_dashboard_uses_vetted_transform_for_main_map_overlays():
    source = (REPOSITORY / 'scripts' /
              'live_multirobot_dashboard.py').read_text(encoding='utf-8')
    assert "'/vetted_transform/leo2_to_leo1'" in source
    assert "'/vetted_alignment_confidence'" in source
    assert "'/shared/apriltag_landmarks'" in source


def test_dashboard_saves_camera_views_and_persistent_local_aruco_overlays():
    collector = (REPOSITORY / 'scripts' /
                 'live_multirobot_dashboard.py').read_text(encoding='utf-8')
    generator = (REPOSITORY / 'scripts' /
                 'generate_recording_dashboard.py').read_text(encoding='utf-8')
    template = (REPOSITORY / 'scripts' /
                'recording_dashboard.html').read_text(encoding='utf-8')
    assert "f'/{name}/aruco_markers'" in collector
    assert "f'camera_{name}.jpg'" in collector
    assert '"cameraEvidence"' in generator
    assert '"localMapEvidence"' in generator
    assert 'id="cameraEvidence"' in template
    assert 'id="localMapEvidence"' in template


def test_dashboard_exposes_ranked_alignment_possibilities():
    aligner = (PACKAGE / "multi_robot_shared_mapping" /
               "map_based_aligner.py").read_text(encoding="utf-8")
    live = (REPOSITORY / 'scripts' /
            'live_dashboard.html').read_text(encoding='utf-8')
    saved = (REPOSITORY / 'scripts' /
             'recording_dashboard.html').read_text(encoding='utf-8')
    assert '"relative_weight"' in aligner
    assert 'id="alignmentCandidates"' in live
    assert 'id="alignmentPossibilities"' in saved


def test_grid_only_lock_is_temporally_vetted_and_tag_gate_is_not_required():
    aligner = (PACKAGE / "multi_robot_shared_mapping" /
               "map_based_aligner.py").read_text(encoding="utf-8")
    launch = (PACKAGE / "launch" / "shared_align.launch.py").read_text(
        encoding="utf-8")
    assert '"grid_only_consensus_cycles", 2' in aligner
    assert '"grid_only_max_ambiguity_ratio", 0.82' in aligner
    assert '"grid_only_min_overlap", 0.55' in aligner
    assert '"grid_only_min_confidence", 0.50' in aligner
    assert '"require_tag_evidence": False' in launch


def test_dashboard_records_the_continuous_candidate_merge():
    collector = (REPOSITORY / 'scripts' /
                 'live_multirobot_dashboard.py').read_text(encoding='utf-8')
    live = (REPOSITORY / 'scripts' /
            'live_dashboard.html').read_text(encoding='utf-8')
    generator = (REPOSITORY / 'scripts' /
                 'generate_recording_dashboard.py').read_text(encoding='utf-8')
    saved = (REPOSITORY / 'scripts' /
             'recording_dashboard.html').read_text(encoding='utf-8')
    assert "'/shared_map_candidate'" in collector
    assert "'candidate_maps'" in collector
    assert 'id="candidateMap"' in live
    assert 'candidate_map_evidence' in generator
    assert 'id="candidateMapEvidence"' in saved
