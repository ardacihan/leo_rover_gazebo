from nav_msgs.msg import OccupancyGrid

from leo_rover_exploration.camera_coverage import CameraCoverageTracker


def test_long_connected_wall_is_segmented_for_multiple_viewpoints():
    message = OccupancyGrid()
    message.info.width = 200
    message.info.height = 10
    message.info.resolution = 0.05
    grid = [0] * (message.info.width * message.info.height)
    for column in range(message.info.width):
        grid[5 * message.info.width + column] = 100
    message.data = grid

    tracker = CameraCoverageTracker()
    clusters = tracker.unobserved_clusters(
        message, min_cells=5, max_cells=30)

    assert len(clusters) >= 6
    assert max(cluster['n_cells'] for cluster in clusters) <= 30
    assert sum(cluster['n_cells'] for cluster in clusters) >= 195
