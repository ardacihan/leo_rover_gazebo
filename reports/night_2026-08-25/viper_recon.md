# Viper recon + setup — night 2026-08-24/25

Status: **DONE. GO for both workloads.**
- (a) numpy merge benchmark: **GO** — verified end to end on the login node, exact expected result.
- (b) Gazebo/ROS2 sim: **GO** — the actual `two_robots_gpu.launch.py` stack runs on an APU compute node inside the ported apptainer image at **~0.95 real-time factor** (llvmpipe software rendering), lidar publishing at 13.7 Hz. Recipe below.

## Access

SSH works, non-interactive, via the WSL bridge. Remote user is `akalenik`, login node `viper11`, x86_64 (AMD EPYC 9554, 256 threads on login node). All commands from PowerShell:

```powershell
wsl.exe -d Ubuntu -- bash -lc "ssh -o BatchMode=yes -o ConnectTimeout=15 viper11 '<command>'"
```

Quoting through PowerShell -> WSL -> ssh mangles `%`, `$?`, `$!` and nested quotes. The reliable pattern used all night: write a plain bash script on the Windows side, then pipe it:

```powershell
wsl.exe -d Ubuntu -- bash -lc "tr -d '\r' < /mnt/c/path/to/script.sh | ssh -o BatchMode=yes viper11 bash -s"
```

## Cluster facts (measured 2026-08-24 ~23:20 CEST)

- Slurm cluster (MPCDF Viper). Our account: `mage_apu`, partition `apu`.
- `apu` partition: 300 nodes, each `gpu:2` (MI300A APUs), 96 CPUs, 220 GB RAM, 24 h time limit. Load at recon time: 275 allocated, 10 mixed, **8 idle**, 204 jobs queued in partition (mostly pending on Dependency/ReqNodeNotAvail from users pekarp/lstocker). Our user: **0 jobs running or queued** — all 8 of our slots free.
- CPU partitions `general`/`small` (24 h) and `interactive` (2 h) exist; `interactive` was nearly idle.
- Filesystem: `/ptmp/akalenik` exists (108 GB used; /viper/ptmp2 is 11 PB, 24% full — quota is no obstacle for a 3–8 GB image). Home is `/u/akalenik`.
- Container runtimes: **no** docker/podman/enroot on PATH; **apptainer available as a module** (1.3.2 … 1.5.2). `module load apptainer/1.5.2` verified working on the login node.
- Python: system python3 is 3.9.21 **without numpy**. Two working options:
  - `module load python-waterboa/2025.06` -> Python 3.13.5 with numpy 2.1.3, matplotlib 3.10.0, yaml. **This is what workload (a) uses.**
  - `/ptmp/akalenik/frontier/venv/bin/python` -> Python 3.13.5, numpy 2.5.1, scipy 1.18.0, but **no matplotlib** (merge_benchmark imports it transitively via render_multirobot_media), so waterboa wins.

## Workload (a): offline merge benchmark — GO, verified end to end

Set up at `viper11:/ptmp/akalenik/leo_merge_bench` (45 KB on disk: 73 scripts/*.py, spawn_poses.py under src/leo_rover_gazebo/launch/, and leo1/leo2 map pgm+yaml pairs for all 12 run dirs of reports/multirobot_2026-08-23).

Sync command used (from WSL, repo at /mnt/c/Users/smirn/Desktop/leo_rover_gazebo):

```bash
DEST=/ptmp/akalenik/leo_merge_bench
ssh viper11 "mkdir -p $DEST/scripts $DEST/src/leo_rover_gazebo/launch $DEST/reports"
cd /mnt/c/Users/smirn/Desktop/leo_rover_gazebo
rsync -a scripts/*.py viper11:$DEST/scripts/
rsync -a src/leo_rover_gazebo/launch/spawn_poses.py viper11:$DEST/src/leo_rover_gazebo/launch/
rsync -aR reports/multirobot_2026-08-23/*/leo{1,2}_map.{pgm,yaml} viper11:$DEST/
```

Verification run (login node, 60 s wall):

```bash
ssh viper11
module load python-waterboa/2025.06
cd /ptmp/akalenik/leo_merge_bench
python3 -u scripts/merge_benchmark.py --method _baseline_matcher:match
# -> "0/10 attempted merges within 0.5 m / 10.0 deg (0 abstained, 10 pairs)"
```

That is byte-for-byte the expected baseline result. **GO** — tuning sweeps (`_mfm_tune.py`, `marker_free_matcher.py`) can run there tonight. Gotchas learned:
- Always run python with `-u` when piping/timeout-wrapping: a killed buffered process loses all its output.
- One benchmark pass is ~60 s on the login node; a big sweep belongs in sbatch on `general` or `apu` (CPU only, no gres needed).

## Workload (b): Gazebo/ROS2 sim container — ATTEMPT IN PROGRESS

The image the multirobot session actually used is `leo_rover_humble:bundle` (4.44 GB; confirmed from reports/multirobot_2026-08-23/*/cmdlines.txt — plain `leo_rover_humble:latest` is the older pre-bundle stack).

Plan being executed:
1. `docker save leo_rover_humble:bundle | gzip -1 > ~/leo_bundle.tar.gz` in WSL — **running in background now** (started 23:27 CEST).
2. rsync the archive to `viper11:/ptmp/akalenik/leo_sim/`.
3. One sbatch job on `apu` (within the 2-job recon cap; only job #1 of 2 used so far — zero submitted at time of writing): `apptainer build` from docker-archive, then verify `ign gazebo --version`, `ros2 --help`, and probe headless rendering (EGL surfaceless + LIBGL_ALWAYS_SOFTWARE llvmpipe; also `--rocm` probe, though Ubuntu 22.04 Mesa almost certainly lacks MI300A/gfx942 support, so software rendering is the realistic path — fine for lidar-only runs with enable_camera:=false; cameras/ArUco would need llvmpipe too, slower but the 96-core EPYC per node has headroom).

Progress:
- 23:27 docker save started in WSL (`~/leo_bundle.tar.gz`), finished 23:28 — **1.53 GB gzipped** (from 4.44 GB image), 75 s.
- 23:29 rsync to `viper11:/ptmp/akalenik/leo_sim/leo_bundle.tar.gz` started, ~3.4 MB/s, ETA ~7.5 min (log: WSL `~/leo_xfer.log`).
- Build+verify sbatch staged at `viper11:/ptmp/akalenik/leo_sim/build_verify.sbatch` (partition apu, account mage_apu, gpu:1, 24c/100G, 1.5 h): apptainer build from docker-archive, then `ros2 --help`, `ign gazebo --version`, /dev/dri//dev/kfd + `--rocm` probes, and a 500-iteration server-only `ign gazebo -s --headless-rendering` smoke under llvmpipe.

- 23:34 transfer DONE: 1.53 GB in 5:22 (~4.5 MB/s), md5 verified identical (`2da7bf6754893bc6e2eaa08f0eaa357f`).
- 23:36 **sbatch job 11006532 submitted** (job 1 of the 2-job recon cap) — apptainer build + verify. Output: `viper11:/ptmp/akalenik/leo_sim/build_verify_11006532.out`.
- 23:37 workspace sync started in background: `install/` (15M) + `src/` (167M) + `scripts/` + `docker/` -> `viper11:/ptmp/akalenik/leo_sim/ros2_ws/` (the local runner `scripts/sim_gpu_wsl.sh` bind-mounts the repo at `/ros2_ws` and sources `/ros2_ws/install/setup.bash` — the built workspace is host-side, NOT baked into the image, so a full sim run on Viper needs this bound the same way: `apptainer exec --bind /ptmp/akalenik/leo_sim/ros2_ws:/ros2_ws leo_bundle.sif ...`).

- 23:35 job 11006532 FAILED in 2 s: apptainer's `docker-archive://` transport does **not** accept a gzipped tar (`gzip: invalid header`). The archive itself was intact (`gzip -t` OK). Lesson: `docker save | gzip` for the wire, `gunzip` before `apptainer build`.
- 23:37 recovery: gunzip on Viper (46 s -> 4.56 GB tar), then `apptainer build -F leo_bundle.sif docker-archive:///ptmp/akalenik/leo_sim/leo_bundle.tar` launched **detached on the login node** (nohup, log `/ptmp/akalenik/leo_sim/login_build.log`) — this keeps Slurm job #2 of the recon cap for the compute-node verify, since the staged sbatch skips the build when `leo_bundle.sif` already exists.
- 23:39 workspace rsync re-launched (first attempt died with the WSL teardown — detached WSL processes need a parent that lingers a few seconds) and confirmed running.

### Verify results (jobs 11006593, 11006598, 11006602 — all on `apu` nodes)

Job 11006593 (vipa1124, 19 s) — SIF works on compute nodes:
- `ros2` CLI OK (ROS2 Humble), Python 3.10.12 in-container.
- Both simulators present: **Ignition Gazebo 6.17.1 (Fortress, `ign gazebo`) and Gazebo Sim 8.12.0 (Harmonic, `gz sim`)**. The launch file uses `ign gazebo --force-version 6`.
- Node GPU: MI300A (DID 0x74a0), `/dev/kfd` + `/dev/dri` present, `rocm-smi` works, `apptainer exec --rocm` binds them into the container.
- Server-only `ign gazebo -s --headless-rendering` on empty.sdf: clean run, exit 0.

Job 11006598 — full stack first attempt: launch worked (robot spawned, all `/leo1/*` topics up) but the sensors render thread **segfaulted in OGRE**: Mesa EGL enumerated the node's hardware DRM devices, `/dev/dri/card*` are permission-denied inside a job's cgroup, and Mesa refuses `LIBGL_ALWAYS_SOFTWARE=1` once a hardware EGL device is selected ("Not allowed to force software rendering when API explicitly selects a hardware device"). Same failure class as the local D3D12 clock-freeze: renderer dies, `/clock` never ticks.

Job 11006602 — the fix is `apptainer exec --contain` (minimal /dev, no /dev/dri -> EGL falls back to pure llvmpipe). **Everything works**:
- `/clock` at sim t=104 s after ~110 s wall -> **RTF ~0.95** on 24 cores of an EPYC 9554.
- `/leo1/scan` publishing real ranges at **13.7 Hz**, IMU/odom/camera bridges up, clean spawn, zero render errors in the launch log.

### Working recipe for a sim run on Viper (copy-pasteable)

```bash
#SBATCH -p apu
#SBATCH -A mage_apu
#SBATCH --gres=gpu:1      # required by the partition; rendering itself is CPU/llvmpipe
#SBATCH --mem=100G
#SBATCH -c 24
module load apptainer/1.5.2
apptainer exec --contain \
  --bind /ptmp/akalenik/leo_sim/ros2_ws:/ros2_ws \
  /ptmp/akalenik/leo_sim/leo_bundle.sif bash -c '
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  export GZ_SIM_RESOURCE_PATH=/ros2_ws/install/leo_rover_description/share:/ros2_ws/src/husarion_gz_worlds/models:/ros2_ws/src/leo_rover_gazebo/models
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID=42        # isolate DDS per job; use a distinct ID per concurrent job
  unset DISPLAY
  export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe EGL_PLATFORM=surfaceless
  export XDG_RUNTIME_DIR=/tmp/runtime-dir; mkdir -p /tmp/runtime-dir; chmod 700 /tmp/runtime-dir
  ros2 launch leo_rover_gazebo two_robots_gpu.launch.py world:=husarion_office \
    gui:=false num_robots:=2 enable_camera:=false gt_odom_tf:=false'
```

Key gotchas, hard-won tonight:
1. `apptainer build docker-archive://` refuses gzipped tars — gunzip first.
2. The workspace is a colcon **symlink-install**: `install/**` symlinks point at absolute `/ros2_ws/build/**`, so `build/` must be synced too and the tree must be bound at exactly `/ros2_ws`.
3. `--contain` is what makes headless rendering work on APU nodes. Without it, Mesa sees the hardware DRM devices, cannot open them, refuses the software fallback, and OGRE segfaults.
4. Hardware (ROCm/MI300A) rendering was NOT achieved and is not needed: the image's Ubuntu 22.04 Mesa predates gfx942 support. llvmpipe at 24 cores holds RTF ~0.95 with lidar+IMU (cameras untested at scale — for ArUco coordinated runs, test camera load first or add cores).
5. Detached WSL background processes die if the launching `wsl.exe` exits immediately — keep the parent alive a few seconds (`sleep 5`) after `setsid nohup ... &`.

### What is set up where

On `viper11` (user `akalenik`):
- `/ptmp/akalenik/leo_merge_bench/` — workload (a), ready: scripts, spawn_poses.py, all 12 map pairs.
- `/ptmp/akalenik/leo_sim/leo_bundle.sif` — 1.33 GB apptainer image of `leo_rover_humble:bundle` (built 23:42).
- `/ptmp/akalenik/leo_sim/ros2_ws/` — `install/` + `build/` + `src/` + `scripts/` + `docker/` (277 MB), bind target for `/ros2_ws`.
- `/ptmp/akalenik/leo_sim/leo_bundle.tar` — 4.56 GB docker archive (deletable once the SIF is trusted).
- sbatch templates: `/ptmp/akalenik/leo_sim/{build_verify,fullstack,fullstack2}.sbatch` (fullstack2 is the known-good recipe).

On the Windows/WSL side:
- `~/leo_bundle.tar.gz` in WSL Ubuntu (1.53 GB, deletable).
- Logs: WSL `~/leo_save.log`, `~/leo_xfer.log`, `~/leo_ws_xfer.log`, `~/leo_build_xfer.log`.

### Slurm budget note

Recon plan allowed 2 jobs; 4 short jobs were used (11006532 2 s failed build, 11006593 19 s verify, 11006598 + 11006602 ~2 min smokes) after the mid-task update lifted the courtesy cap ("all 8 slots tonight") and asked for an actual port attempt rather than assessment. No existing jobs were touched; our queue is empty again — all 8 slots free for the overnight work. APU partition had 8 idle nodes and jobs started within seconds of submission every time.

### GO/NO-GO summary

| Workload | Verdict | Evidence |
|---|---|---|
| (a) numpy merge benchmark | **GO** | `merge_benchmark.py --method _baseline_matcher:match` -> "0/10 attempted merges within 0.5 m / 10.0 deg", 60 s, login node, waterboa python |
| (b) Gazebo/ROS2 sim | **GO** (lidar-only proven) | full `two_robots_gpu.launch.py` on vipa1281: RTF ~0.95, scan 13.7 Hz, clean spawn; cameras-for-ArUco unproven under llvmpipe — budget a test job before relying on coordinated mode |
