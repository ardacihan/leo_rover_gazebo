# Calibration Guide

The goal is not to manually calibrate every internal camera parameter. The RealSense driver already provides factory intrinsics and the depth-to-colour relationship. The important project-specific values are the geometry between the rover base, LiDAR, camera, wheels, and doorway.

Use the URDF and live TF tree as the first estimate. Measure only the values that depend on how the sensors were mounted on this rover. The supplied tools are read-only: they print recommendations and never publish a replacement transform.

## Safety before measurement

Stop autonomous navigation and publish zero velocity. Keep the rover on a level floor. Do not run the interactive odometry calibration near stairs, people, cables, or furniture. A second person should guard the hardware during any motion.

## 1. Determine the base-frame origin

First identify the frame used by Nav2:

- simulator: `leo1/base_link`;
- starting real profile: `base_footprint`.

View the existing transform tree:

```bash
ros2 run tf2_tools view_frames
ros2 run leo_nav2_exploration tf_snapshot base_footprint laser_frame
ros2 run leo_nav2_exploration tf_snapshot base_footprint camera_link
```

Use the actual frame names shown by `view_frames`. If the model already contains plausible fixed transforms, preserve them as the initial values. Do not launch another static transform on top of them.

## 2. Measure the rover footprint

Measure horizontally from the base-frame origin to the furthest rigid point at approximately obstacle height. Include wheels, wheel brackets, bumpers, and anything that can strike a vertical panel. Ignore flexible cables that are safely inside the body envelope.

Record four numbers:

```text
front: origin to furthest front point
rear:  origin to furthest rear point
left:  origin to furthest left point
right: origin to furthest right point
```

The values do not need to be symmetric. The starting assumption is 0.21 m in every direction, giving a 0.42 m square.

Generate the Nav2 footprint:

```bash
ros2 run leo_nav2_exploration footprint_tool \
  --front 0.21 \
  --rear 0.21 \
  --left 0.21 \
  --right 0.21 \
  --padding 0.01 \
  --door-width 0.78
```

The tool reports physical width, padded width, circumscribed radius, total doorway margin, and centred margin per side. A positive mathematical margin means only that the footprint fits geometrically; real operation still needs localization, control, and sensor error allowance.

Copy the footprint string into both local and global costmaps. A 0.01 m padding is a reasonable starting value after the physical outline has been measured accurately. Do not hide measurement uncertainty by adding a very large footprint padding because that can make valid doors impossible to plan through.

## 3. Measure clear doorway width

Measure the narrowest rigid distance between the two vertical door jambs or blocking planes at the height where the rover is widest. Do not use nominal door size. If trim, hinges, or a board protrudes, measure between those protrusions.

Compare the clear width with the padded footprint using `footprint_tool`. For reliable tests, begin with a passage that leaves at least about 0.10 m total geometric margin. The included simulation fixture has a 0.78 m opening and a 0.44 m padded starting footprint, leaving 0.34 m total geometric margin.

## 4. LiDAR translation and yaw

The highest-value LiDAR quantities for 2D SLAM are X, Y, and yaw relative to the base frame. Z should still be correct, but a moderate Z error does not distort a planar scan as strongly as a wrong yaw or horizontal offset.

### Use live defaults first

```bash
ros2 run leo_nav2_exploration tf_snapshot base_footprint laser_frame
```

Record the translation and yaw. Confirm the transform has exactly one owner using `view_frames` and the running launch/process list.

### Physical direction test

Place one broad, flat board approximately 1.0 m from the LiDAR centre. Keep it vertical and perpendicular to the rover's forward direction. Remove nearby objects from the analysis sector. Use the raw scan, not `/scan_filtered`, because this test is measuring the sensor itself.

```bash
ros2 run leo_nav2_exploration lidar_board_calibration \
  --topic /scan \
  --expected-angle-deg 0 \
  --half-width-deg 30 \
  --known-distance 1.0 \
  --samples 30
```

For the simulator, use `--topic /leo1/scan`.

The script fits the board line across multiple scans. It reports:

- board normal angle in the scan;
- recommended base-to-LiDAR yaw;
- perpendicular LiDAR-to-board distance;
- line-fit RMS;
- optional range scale from tape distance.

Repeat with the board on the left at +90 degrees and on the right at -90 degrees. The front/left/right signs must follow the ROS convention used by the scan. A 180-degree error is not a tuning issue; correct the sensor transform or driver convention.

Measure LiDAR X and Y from the base origin to the LiDAR optical centre. Use a square and plumb line rather than the sensor housing edge. Enter the corrected transform in the existing URDF or single static-transform source. Do not add an additional publisher.

The overlay's LaserScan box filter removes returns inside the rover envelope. Keep the raw topic for calibration and the filtered topic for SLAM, costmaps, the velocity guard, and Collision Monitor.

## 5. Camera translation, roll, and pitch

Factory intrinsics should normally remain untouched. Project-specific measurements are the camera origin relative to the base and the camera's mounting orientation.

### Read existing transforms

The RealSense driver often publishes a chain ending in an optical frame. Inspect it:

```bash
ros2 run tf2_tools view_frames
ros2 run leo_nav2_exploration tf_snapshot base_footprint camera_link
ros2 run leo_nav2_exploration tf_snapshot base_footprint camera_depth_optical_frame
```

Frame names vary. The VoxelLayer needs a valid transform from the `PointCloud2.header.frame_id` to the local costmap frame. Do not assume the point cloud uses `camera_link`.

### Floor-plane estimate

Place the rover on a flat floor with a large unobstructed floor patch visible. Keep the rover still. Run:

```bash
ros2 run leo_nav2_exploration camera_floor_calibration \
  --topic /camera/camera/depth/color/points \
  --frames 8 \
  --max-floor-tilt-deg 50
```

Simulator:

```bash
ros2 run leo_nav2_exploration camera_floor_calibration \
  --topic /leo1/camera/points \
  --frames 8
```

The script fits a floor plane constrained toward the expected optical-frame upward direction. It reports camera optical-origin height, roll, downward pitch, inlier fraction, and RMS. A low inlier fraction means the selected points are dominated by walls, furniture, invalid depth, or non-flat floor; repeat rather than copying the result.

Remember that ROS optical axes are normally X right, Y down, Z forward. The reported optical pitch is not always numerically identical to the `camera_link` pitch because the fixed optical-frame rotation is part of the TF chain.

Measure camera X, Y, and Z from the base origin to the optical centre. For collision detection, Z and pitch matter because they determine which floor and obstacle heights enter the point cloud. Update the existing URDF or transform source and verify with `tf_snapshot`.

## 6. Odometry distance and rotation scale

Bad odometry makes walls smear, rooms duplicate, and loop closure fail. Calibrate the existing odometry source rather than creating a second `odom -> base` broadcaster.

### Straight distance

Mark a straight 2.0 m line on the floor. Stop Nav2. Start the observer:

```bash
ros2 run leo_nav2_exploration odom_calibration \
  --topic /wheel_odom \
  --mode linear \
  --actual 2.0
```

Press Enter at the start mark, manually drive as straight as possible, stop exactly at the end mark, then press Enter. Repeat in both directions several times. The reported scale is `actual / odometry-reported`. Apply it in the wheel-odometry source, wheel radius, ticks-per-revolution setting, or estimator—not in SLAM.

### Rotation

Mark the initial heading and rotate exactly 360 degrees at low speed:

```bash
ros2 run leo_nav2_exploration odom_calibration \
  --topic /wheel_odom \
  --mode angular \
  --actual 360
```

Repeat clockwise and counter-clockwise. A consistent angular scale error indicates wheel-separation or kinematic calibration. Large direction-dependent differences indicate slip, unequal wheel behavior, timestamp problems, or a model mismatch. Avoid repeated high-rate in-place spins during mapping.

## 7. One-command report

After building the overlay:

```bash
./scripts/calibration_report.sh /ros2_ws real_root 0.78 laser_frame camera_link
```

This prints raw/filtered topic availability, current LiDAR and camera transforms, and a starting footprint calculation. It does not replace the individual board, floor, or odometry experiments.

## 8. Acceptance checks after calibration

Before exploration, require all of the following:

1. Stationary robot for 60 seconds: pose and map do not drift visibly.
2. Slow 360-degree turn: walls return to the same cells without a second copy.
3. Straight out-and-back: map alignment remains stable.
4. Raw scan shows the environment and filtered scan removes only rover-mounted returns.
5. A manual Nav2 goal crosses the doorway in both directions.
6. The local footprint stays centred on the model in RViz.
7. Point-cloud obstacles align with LiDAR/walls when the VoxelLayer is enabled.
8. Exactly one final command publisher exists.

If any stationary or TF test fails, stop. Planner tuning cannot compensate for inconsistent transforms or odometry.
