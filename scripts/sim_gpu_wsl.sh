#!/usr/bin/env bash
# Start Leo Rover Gazebo with GPU OpenGL (Docker in WSL + WSLg).
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PATCHED_OGRE="${ROOT}/docker/patched/RenderSystem_GL3Plus.so"
OGRE_MOUNTS=()

# Git-bash (MSYS) rewrites every /abs/path argument into C:\Program Files\Git\...
# before docker.exe sees it, which breaks -v and -e. Disable that and hand
# docker a native Windows path for the workspace mount instead.
HOST_ROOT="${ROOT}"
if command -v cygpath >/dev/null 2>&1; then
  export MSYS_NO_PATHCONV=1
  HOST_ROOT="$(cygpath -w "${ROOT}")"
fi

if [[ -f "${PATCHED_OGRE}" ]]; then
  OGRE_MOUNTS+=(
    -v "${HOST_ROOT}/docker/patched/RenderSystem_GL3Plus.so:/usr/lib/x86_64-linux-gnu/OGRE-Next/RenderSystem_GL3Plus.so:ro"
    -v "${HOST_ROOT}/docker/patched/RenderSystem_GL3Plus.so.2.2.5:/usr/lib/x86_64-linux-gnu/OGRE-Next/RenderSystem_GL3Plus.so.2.2.5:ro"
  )
  echo "Using patched Ogre GPU renderer from docker/patched/"
else
  echo "No patched Ogre found. Run: ./scripts/build_ogre_wsl_gpu.sh"
fi

docker stop leo_sim 2>/dev/null || true
docker rm leo_sim 2>/dev/null || true

# Mesa's d3d12 driver (the only GPU path here -- Docker Desktop forwards CUDA
# but ships no NVIDIA EGL/GLX ICD) is reachable through surfaceless EGL only.
# With DISPLAY set, Ogre takes the GLX/X11 path instead and silently lands on
# llvmpipe. So a headless run must present no display at all.
SIM_DISPLAY="${DISPLAY:-:0}"
if [[ "${GUI:-true}" != "true" ]]; then
  SIM_DISPLAY=""
fi

docker run -d --name leo_sim \
  --gpus all \
  --device=/dev/dxg \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all,compute,utility,graphics \
  -e DISPLAY="${SIM_DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_INDIRECT=0 \
  -e LIBGL_ALWAYS_SOFTWARE=0 \
  -e GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}" \
  -e XDG_RUNTIME_DIR=/tmp/runtime-dir \
  -e ROS2_WS=/ros2_ws \
  -e GZ_VERSION=harmonic \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /usr/lib/wsl:/usr/lib/wsl:ro \
  -v "${HOST_ROOT}:/ros2_ws" \
  "${OGRE_MOUNTS[@]}" \
  --entrypoint bash \
  "${LEO_IMAGE:-leo_rover_humble:bundle}" \
  -lc 'export LD_LIBRARY_PATH=/usr/lib/wsl/lib:${LD_LIBRARY_PATH} && \
       source /opt/ros/humble/setup.bash && \
       source /ros2_ws/install/setup.bash && \
       export GZ_SIM_RESOURCE_PATH=/ros2_ws/install/leo_rover_description/share:/ros2_ws/src/husarion_gz_worlds/models:/ros2_ws/src/leo_rover_gazebo/models && \
       mkdir -p /tmp/runtime-dir && chmod 700 /tmp/runtime-dir && \
       if [[ -f /ros2_ws/docker/patched/RenderSystem_GL3Plus.so ]]; then \
         mkdir -p /usr/local/share/leo_rover_gazebo && \
         touch /usr/local/share/leo_rover_gazebo/ogre_wsl_gpu_patched; \
       fi && \
       ros2 launch leo_rover_gazebo two_robots_gpu.launch.py world:='"${WORLD:-husarion_office}"' gui:='"${GUI:-true}"' num_robots:='"${NUM_ROBOTS:-1}"' enable_camera:='"${ENABLE_CAMERA:-true}"' gt_odom_tf:='"${GT_ODOM_TF:-true}"' sim_speed:='"${SIM_SPEED:-1.0}"

echo "Leo sim started (container: leo_sim)"
echo "Logs: docker logs -f leo_sim"
