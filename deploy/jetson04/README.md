# Rover 4 host configuration

Machine configuration that lives outside the ROS workspace and would otherwise
be lost if the Jetson were reimaged or the SD card replaced.

## `lidar-tf.service.d-override.conf`

The stock `lidar-tf.service` publishes `base_link -> laser_frame` with `--y 0`,
but the lidar is physically **0.04 m to the rover's left**. Every map built
under the stock unit carries a 4 cm lateral bias. Two independent measurements
agree on the correction: an operator tape measurement, and the lidar's own view
of the camera mast, which resolves to `y=-0.038` against a tape reading of
`-0.040`.

The stock unit is also *named* for an RPLIDAR S3, which is wrong. The driver
reports a 5 kHz sample rate and an S3 samples at 32 kHz, so this is a C1-class
unit; 5 kHz at 10 Hz gives the ~510 points per scan actually seen on `/scan`.
The override corrects the description too.

Install:

```bash
sudo mkdir -p /etc/systemd/system/lidar-tf.service.d
sudo cp lidar-tf.service.d-override.conf \
        /etc/systemd/system/lidar-tf.service.d/override.conf
sudo systemctl daemon-reload
sudo systemctl restart lidar-tf
```

Verify — the second command must report `[0.077, 0.040, 0.246]`:

```bash
systemctl show lidar-tf -p ExecStart
ros2 run tf2_ros tf2_echo base_footprint laser_frame
```

A drop-in rather than an edit of the unit file, so a package update cannot
silently revert it and `rm` restores stock behaviour. Confirmed to survive a
reboot.

Because the boot unit now publishes the correct transform, run
`safe_mapping.launch.py` with `publish_lidar_tf:=false`. Setting it true while
`lidar-tf.service` is active gives `laser_frame` two parents and the TF tree
becomes ambiguous.
