"""Source contracts for simulation and physical multi-rover launch safety."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def test_collaborative_explorer_has_explicit_physical_runtime_overrides():
    source = (PACKAGE / 'launch' / 'collab_explore.launch.py').read_text(
        encoding='utf-8')
    assert "DeclareLaunchArgument('use_sim_time', default_value='true')" in source
    assert "DeclareLaunchArgument('base_frame_suffix', default_value='base_link')" in source
    assert "DeclareLaunchArgument('command_topic_suffix', default_value='cmd_vel')" in source
    assert "'cmd_vel_topic': f'/{ns}/{command_suffix}'" in source
    assert "'robot_base_frame': f'{ns}/{base_suffix}'" in source
    assert "'shared_alignment_topic'" in source
    assert "f'/{ns}/shared_map_raw'" in source
    assert "'landmarks_topic': f'/{ns}/aruco_markers'" in source
