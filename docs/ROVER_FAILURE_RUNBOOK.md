# Rover Failure Runbook (jetson-04 / rover 4, jetson-02 / rover 2)

Every failure below was hit live on 2026-08-25 and fixed. Ordered by how
often each one bites. Diagnose top-down; most incidents are one of the
first three.

## 0. Quick health matrix (run this first)

```bash
# On the jetson (rover 4 shown; rover 2: domain 2, root topics)
ssh leo4
bash ~/leo_nav2_ws/preflight.sh

# On the SBC (via the jetson; password raspberry)
sshpass -p raspberry ssh pi@10.0.0.1 '
  vcgencmd measure_temp; vcgencmd get_throttled     # thermal FIRST
  ls /dev/shm | grep -c fastrtps                    # want 0
  journalctl --user -u uros-agent --since "60 seconds ago" | grep -ci "hard timeout"
  uptime'
```

Interpretation: `get_throttled` != 0x0 with live bits (0x7 low nibble) →
§2. SHM count > 0 → §1. Hard timeouts > 0/min with SHM 0 and no throttle
→ §3. All clean but no telemetry on the jetson → §5/§6.

## 1. SHM starvation trap (micro-ROS session death, classic form)

**Symptom:** all `/rob_2/firmware/*` silent, SBC pings fine, agent ~106%
CPU, journal loops "session hard timeout". `ls /dev/shm | grep -c
fastrtps` large (~70).

**Cause:** ros-nodes + uros-agent share `--ipc=host`; FastDDS SHM
segments accumulate across restarts; past ~70 the enumeration is slow
enough that the STM32 keepalive is missed.

**Permanent fix (deployed on both rovers):** UDP-only FastDDS profile via
`FASTRTPS_DEFAULT_PROFILES_FILE=/etc/ros/fastdds_udp_only.xml` in both
service overrides, plus `clean-shm.conf` ExecStartPre purge on ros-nodes.

**Recovery (~2 min, no reboot):**
```bash
systemctl --user stop uros-agent ros-nodes
podman ps -q | xargs -r podman stop -t 5
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
systemctl --user start ros-nodes; sleep 8; systemctl --user start uros-agent
```
Then on the jetson: `ros2 daemon stop` (the CLI caches the dead graph).

## 2. Thermal starvation trap (same symptom, different cause)

**Symptom:** hard timeouts persist after §1 recovery; agent CPU LOW
(~2%), SBC load 20+, `vcgencmd get_throttled` shows live bits (e.g.
0xe0006) and `measure_clock arm` reads 600 MHz (normal 1500 MHz).
Observed at 85.7 °C.

**Cause:** sustained session churn heats the Pi to the soft temp limit →
CPU capped → keepalives miss → more churn → more heat. Self-sustaining.

**Fix:** `systemctl --user stop uros-agent ros-nodes`, wait for
< ~70 °C (5–10 min), restart. Check heatsink/fan/airflow. No software
restart works while throttled.

## 3. Discovery-churn starvation + the ros-nodes question

**Symptom:** hard timeouts at full CPU speed, SHM 0, WiFi kworker busy.

**Cause:** every session re-create triggers full DDS discovery with ~25
jetson participants; over congested WiFi it is slow enough to miss the
keepalive.

**Fix (deployed):** `interfaceWhiteList` in both machines' FastDDS
profiles — SBC: 127.0.0.1 + 10.0.0.1; jetson ws copy: 127.0.0.1 +
10.0.0.76. Firmware/stack DDS never touches WiFi (the wired rover LAN
carries it; laptop access is ssh/rosbridge TCP, unaffected).

**Nuclear option that WORKS (validated on the 2026-08-25 evening runs):**
run without `ros-nodes` entirely. The STM32 publishes wheel_odom,
battery(+averaged), imu agent-direct; leo-nav-bridge consumes them and
the full exploration stack runs fine. If ros-nodes churns the session,
`systemctl --user stop ros-nodes` and carry on.

## 4. Boot auto-drive traps (rover drives itself)

`leo-nav.service` (crash-loops) and `leo-ros.service`
(`real_rover.launch.py mission:=exploration` → exploration_supervisor
DRIVES THE ROVER ~30 s after boot). Both are disabled on jetson-04, but
**leo-ros comes back at boot anyway** (a dependency pulls it in; mask is
impossible — unit file lives directly in /etc/systemd/system).

**After ANY jetson reboot, immediately:**
```bash
sudo systemctl stop leo-ros
pkill -f 'exploration_superviso[r]'; pkill -f 'stuck_recover[y]'
```

**Emergency stop that works even when the ROS graph is unprobeable:**
restart uros-agent on the SBC — the micro-ROS session drop trips the
STM32 motor failsafe.

## 5. leo-nav-bridge wedge (odom TF vanishes)

**Symptom:** `/wheel_odom` has a registered publisher but no data;
odom→base_footprint TF missing; SLAM logs "Message Filter dropping";
costmaps log "Timed out waiting for transform"; Nav2 activation hangs.

**Fix:** `sudo systemctl restart leo-nav-bridge` — TF returns within
seconds. Check this BEFORE blaming anything else when TF is missing.

## 6. Nav2 lifecycle bringup "Failed to change state"

**Symptom:** lifecycle_manager aborts on a different node each attempt
(controller_server most often). The node is usually FINE — it just
answered slower than the manager's service timeout (boot load, missing
TF making costmap activation wait).

**Fix:** don't loop full restarts. Check states and activate manually:
```bash
for n in controller_server smoother_server planner_server behavior_server \
         velocity_smoother waypoint_follower bt_navigator; do
  ros2 lifecycle get /$n
  ros2 lifecycle set /$n activate   # if inactive (configure first if unconfigured)
done
```
Also: start recorders/aruco AFTER Nav2 is active — their bringup-time
load caused repeated activation timeouts.

## 7. pkill self-kill (fake SSH drops, half-done restarts)

`pkill -f 'nav2'` matches the invoking shell's OWN command line and
every path containing `leo_nav2_ws` — it kills the ssh session (exit
255 mid-command) or the restart script itself. Many "network flakes"
were this.

**Rule:** bracket one character AND target binary paths:
`pkill -9 -f '/opt/ros/humble/lib/nav[2]'`, `pkill -f 'tf_freshener.p[y]'`.
Multi-step restarts belong in a detached remote script
(`~/leo_nav2_ws/full_restart.sh`), not inline ssh commands.

## 8. TF timing (odom stamps vs Jetson clock)

Firmware odom stamps drift behind the Jetson clock → explore's
getRobotPose fails "extrapolation into the future" ~35 min into a run
(killed frontier goals), and marker/cloud transforms fail.

**Fix (deployed, validated: zero SLAM message drops since):**
`tf_freshener.py` re-stamps BOTH map→odom and odom→base_footprint with
Jetson-now at 20 Hz. Exactly one instance may run
(`pgrep -f 'tf_freshener.p[y]'`).

## 9. Probes lie; attached consumers don't

Fresh `ros2 topic hz/echo` probes routinely see nothing (best-effort
QoS + DDS participant churn) while the topic is flowing fine. Before
declaring a topic dead: check an attached consumer's log, run a
5–10 s Python subscriber, or `ros2 daemon stop` and retry. Publisher
count in `ros2 topic info` proves registration, NOT data flow (see §5).

## 10. Scan filter notes

- The overlay launch starts its own scan_to_scan_filter_chain; the
  respawn wrapper `run_scanfilter.sh` duplicates it → /scan_filtered at
  20 Hz instead of 10. Run ONE: kill both, start the wrapper.
- laser_filters sporadically dies with "int64_t overflow" — that is why
  the wrapper exists. If /scan_filtered has 0 publishers the collision
  monitor holds the robot.

## 11. Power realities

- The jetson and the rover chassis (SBC + STM32 + motors) have SEPARATE
  supplies. The chassis power button does NOT reboot the jetson.
- Battery floor: a "10.0 V" pack brownout-rebooted the jetson within
  ~15 min idle; under 10.3 V resting, don't start a run. Fresh pack
  reads ~11.9 V.
- Firmware telemetry death at ~10.7 V under load in the morning was NOT
  the battery — it was §1.

## 12. Rover 2 (jetson-02) specifics

- ssh alias `leo2`; SBC reachable key-based from jetson-02.
- Domain 2; firmware topics at ROOT (`/firmware/...`, `/cmd_vel`);
  camera under `/rob_2`; no leo-nav-bridge — `odom_relay.py` bridges
  /firmware/wheel_odom → /wheel_odom.
- Lidar: RPLIDAR C1, 460800, /dev/ttyUSB0, frame renamed `laser_frame`
  (static TF base_footprint→laser_frame, z=0.15) so rover-4 configs run
  unchanged. Bringup: `run_lidar.sh`.
- **OPEN BUG:** controller instantly reports "Reached the goal!" and the
  rover never moves; collision monitor logs laser_frame TF timing
  errors. Undebugged — reproduce with `start_stack.sh` + one goal.

## 13. Recording stack (validated, starvation-safe)

Start AFTER Nav2 is active: `run_rec.sh` (map/costmaps/pose/goals npz
@0.5 Hz + 1 Hz 320p camera), aruco via `run_aruco.sh`, and 480p video by
dumping web_video_server:
```bash
curl -s 'http://localhost:8080/stream?topic=/rob_4/camera/color/image_raw&width=854&height=480&quality=70' \
  -o <run_dir>/video_hd/stream.mjpg   # ~1 MB/s; fps param is ignored
ffmpeg -r 25 -i stream.mjpg -c copy out.mp4   # instant, no re-encode
```
The camera's `/compressed` topic has no publisher (plugin not loaded) —
don't subscribe to it.
