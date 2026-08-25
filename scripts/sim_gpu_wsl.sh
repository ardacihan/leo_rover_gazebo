#!/usr/bin/env bash
# Start Leo Rover Gazebo with GPU OpenGL in Docker.
#
# The original wrapper only supported WSLg.  Keep that accelerated path, but
# also support a native Ubuntu/X11 host so the exact same simulation command
# can open Gazebo on the operator's desktop.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PATCHED_OGRE="${ROOT}/docker/patched/RenderSystem_GL3Plus.so"
OGRE_MOUNTS=()
PLATFORM_ARGS=()
DISPLAY_ARGS=()
CONTAINER_ENV=()
IS_WSL=false
[[ -e /dev/dxg ]] && IS_WSL=true

# Git-bash (MSYS) rewrites every /abs/path argument into C:\Program Files\Git\...
# before docker.exe sees it, which breaks -v and -e. Disable that and hand
# docker a native Windows path for the workspace mount instead.
HOST_ROOT="${ROOT}"
if command -v cygpath >/dev/null 2>&1; then
  export MSYS_NO_PATHCONV=1
  HOST_ROOT="$(cygpath -w "${ROOT}")"
fi

if [[ "${IS_WSL}" == "true" ]]; then
  if [[ -f "${PATCHED_OGRE}" ]]; then
    OGRE_MOUNTS+=(
      -v "${HOST_ROOT}/docker/patched/RenderSystem_GL3Plus.so:/usr/lib/x86_64-linux-gnu/OGRE-Next/RenderSystem_GL3Plus.so:ro"
      -v "${HOST_ROOT}/docker/patched/RenderSystem_GL3Plus.so.2.2.5:/usr/lib/x86_64-linux-gnu/OGRE-Next/RenderSystem_GL3Plus.so.2.2.5:ro"
    )
    echo "Using patched Ogre GPU renderer from docker/patched/"
  else
    echo "No patched Ogre found. Run: ./scripts/build_ogre_wsl_gpu.sh"
  fi
else
  echo "Using native Ubuntu NVIDIA/X11 rendering"
fi

docker stop leo_sim 2>/dev/null || true
docker rm leo_sim 2>/dev/null || true

# Mesa's d3d12 driver is the WSL GPU path.  Native Ubuntu instead uses the
# NVIDIA container runtime and the host X11 socket.
SIM_DISPLAY="${DISPLAY:-}"
if [[ -z "${SIM_DISPLAY}" && "${GUI:-true}" == "true" ]]; then
  # Non-interactive shells often do not inherit DISPLAY.  Use the active X
  # socket when there is exactly one (the normal Ubuntu desktop case).
  X_SOCKET="$(find /tmp/.X11-unix -maxdepth 1 -type s -name 'X*' 2>/dev/null | head -1 || true)"
  [[ -n "${X_SOCKET}" ]] && SIM_DISPLAY=":${X_SOCKET##*X}"
fi
if [[ "${GUI:-true}" != "true" ]]; then
  SIM_DISPLAY=""
fi

if [[ "${IS_WSL}" == "true" ]]; then
  PLATFORM_ARGS+=(
    --device=/dev/dxg
    -v /usr/lib/wsl:/usr/lib/wsl:ro
    -p "${DASHBOARD_PORT:-8080}:${DASHBOARD_PORT:-8080}"
  )
  CONTAINER_ENV+=(
    -e GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
    -e LEO_GPU_LIBRARY_PATH=/usr/lib/wsl/lib
  )
else
  # /dev/dri is used by Mesa/GL probing even when rendering lands on NVIDIA.
  PLATFORM_ARGS+=(--network host)
  [[ -d /dev/dri ]] && PLATFORM_ARGS+=(--device=/dev/dri)
  CONTAINER_ENV+=(-e LEO_GPU_LIBRARY_PATH=)
fi

if [[ -n "${SIM_DISPLAY}" ]]; then
  DISPLAY_ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
  HOST_XAUTH="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
  if [[ -f "${HOST_XAUTH}" ]]; then
    DISPLAY_ARGS+=(
      -v "${HOST_XAUTH}:/tmp/leo-xauthority:ro"
      -e XAUTHORITY=/tmp/leo-xauthority
    )
  fi
fi

LEO_IMAGE_NAME="${LEO_IMAGE:-leo_rover_humble:bundle}"
if ! docker image inspect "${LEO_IMAGE_NAME}" >/dev/null 2>&1; then
  if docker image inspect leo_rover_humble:latest >/dev/null 2>&1; then
    LEO_IMAGE_NAME=leo_rover_humble:latest
  else
    echo "FATAL: neither ${LEO_IMAGE_NAME} nor leo_rover_humble:latest exists" >&2
    exit 1
  fi
fi

docker run -d --name leo_sim \
  --gpus all \
  "${PLATFORM_ARGS[@]}" \
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
  "${CONTAINER_ENV[@]}" \
  "${DISPLAY_ARGS[@]}" \
  -v "${HOST_ROOT}:/ros2_ws" \
  "${OGRE_MOUNTS[@]}" \
  --entrypoint bash \
  "${LEO_IMAGE_NAME}" \
  -lc 'if [[ -n "${LEO_GPU_LIBRARY_PATH}" ]]; then export LD_LIBRARY_PATH="${LEO_GPU_LIBRARY_PATH}:${LD_LIBRARY_PATH}"; fi && \
       source /opt/ros/humble/setup.bash && \
       source /ros2_ws/install/setup.bash && \
       export GZ_SIM_RESOURCE_PATH=/ros2_ws/install/leo_rover_description/share:/ros2_ws/src/husarion_gz_worlds/models:/ros2_ws/src/leo_rover_gazebo/models && \
       mkdir -p /tmp/runtime-dir && chmod 700 /tmp/runtime-dir && \
       if [[ -f /ros2_ws/docker/patched/RenderSystem_GL3Plus.so ]]; then \
         mkdir -p /usr/local/share/leo_rover_gazebo && \
         touch /usr/local/share/leo_rover_gazebo/ogre_wsl_gpu_patched; \
       fi && \
       ros2 launch leo_rover_gazebo two_robots_gpu.launch.py world:='"${WORLD:-husarion_office}"' gui:='"${GUI:-true}"' num_robots:='"${NUM_ROBOTS:-1}"' enable_camera:='"${ENABLE_CAMERA:-true}"' gt_odom_tf:='"${GT_ODOM_TF:-true}"

echo "Leo sim started (container: leo_sim)"
echo "Logs: docker logs -f leo_sim"
