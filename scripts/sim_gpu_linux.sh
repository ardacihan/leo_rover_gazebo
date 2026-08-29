#!/usr/bin/env bash
# Start Leo Rover Gazebo with GPU OpenGL on NATIVE Linux (Ubuntu + NVIDIA).
#
# Same contract as scripts/sim_gpu_wsl.sh -- same container name, same mount
# point, same launch arguments -- so scripts/auto_multirobot_run.sh can use
# either one via SIM_LAUNCHER. The difference is the GPU path:
#
#   WSL (sim_gpu_wsl.sh): no NVIDIA EGL/GLX ICD reaches the container, so Ogre
#     runs through Mesa's d3d12 Gallium driver over /usr/lib/wsl. That path
#     reports "D3D12 (NVIDIA ...)", caps out at OpenGL 4.2, and segfaults in
#     libd3d12core.so after 9-13 min of two-camera rendering.
#
#   Native (this script): the NVIDIA container runtime exposes the real driver,
#     so Ogre gets native GL. No /dev/dxg, no /usr/lib/wsl, no patched Ogre,
#     no GALLIUM_DRIVER override.
#
# Requires nvidia-container-toolkit ("docker run --gpus all" must expose
# /dev/nvidia*). Verify after starting with:
#   docker exec leo_sim grep -iE 'GL_VERSION|GL_RENDERER' \
#       /root/.ignition/rendering/ogre2.log
# A line naming the NVIDIA card is the pass; "llvmpipe" means software GL.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -e /dev/dxg || -d /usr/lib/wsl ]]; then
  echo "WARNING: this looks like WSL. Use scripts/sim_gpu_wsl.sh there;" >&2
  echo "         the native NVIDIA GL path does not exist under WSL." >&2
fi

docker stop leo_sim 2>/dev/null || true
docker rm leo_sim 2>/dev/null || true

# Headless must present no display at all: with DISPLAY set, Ogre takes the
# GLX/X11 path and can land on llvmpipe.
SIM_DISPLAY="${DISPLAY:-:0}"
X11_MOUNT=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
if [[ "${GUI:-true}" != "true" ]]; then
  SIM_DISPLAY=""
  X11_MOUNT=()
fi

docker run -d --name leo_sim \
  --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all,compute,utility,graphics \
  -e DISPLAY="${SIM_DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_INDIRECT=0 \
  -e LIBGL_ALWAYS_SOFTWARE=0 \
  -e XDG_RUNTIME_DIR=/tmp/runtime-dir \
  -e ROS2_WS=/ros2_ws \
  -e GZ_VERSION=harmonic \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  "${X11_MOUNT[@]}" \
  -v "${ROOT}:/ros2_ws" \
  --entrypoint bash \
  "${LEO_IMAGE:-leo_rover_humble:bundle}" \
  -lc 'source /opt/ros/humble/setup.bash && \
       source /ros2_ws/install/setup.bash && \
       export GZ_SIM_RESOURCE_PATH=/ros2_ws/install/leo_rover_description/share:/ros2_ws/src/husarion_gz_worlds/models:/ros2_ws/src/leo_rover_gazebo/models && \
       mkdir -p /tmp/runtime-dir && chmod 700 /tmp/runtime-dir && \
       ros2 launch leo_rover_gazebo two_robots_gpu.launch.py world:='"${WORLD:-husarion_office}"' gui:='"${GUI:-true}"' num_robots:='"${NUM_ROBOTS:-1}"' enable_camera:='"${ENABLE_CAMERA:-true}"' gt_odom_tf:='"${GT_ODOM_TF:-true}"

echo "Leo sim started (container: leo_sim)"
echo "Logs: docker logs -f leo_sim"
