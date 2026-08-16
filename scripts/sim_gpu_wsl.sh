#!/usr/bin/env bash
# Start Leo Rover Gazebo with GPU OpenGL (Docker in WSL + WSLg).
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PATCHED_OGRE="${ROOT}/docker/patched/RenderSystem_GL3Plus.so"
OGRE_MOUNTS=()

if [[ -f "${PATCHED_OGRE}" ]]; then
  OGRE_MOUNTS+=(
    -v "${ROOT}/docker/patched/RenderSystem_GL3Plus.so:/usr/lib/x86_64-linux-gnu/OGRE-Next/RenderSystem_GL3Plus.so:ro"
    -v "${ROOT}/docker/patched/RenderSystem_GL3Plus.so.2.2.5:/usr/lib/x86_64-linux-gnu/OGRE-Next/RenderSystem_GL3Plus.so.2.2.5:ro"
  )
  echo "Using patched Ogre GPU renderer from docker/patched/"
else
  echo "No patched Ogre found. Run: ./scripts/build_ogre_wsl_gpu.sh"
fi

docker stop leo_sim 2>/dev/null || true
docker rm leo_sim 2>/dev/null || true

docker run -d --name leo_sim \
  --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all,compute,utility,graphics \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_INDIRECT=0 \
  -e MESA_GL_VERSION_OVERRIDE=4.2 \
  -e MESA_GLSL_VERSION_OVERRIDE=420 \
  -e XDG_RUNTIME_DIR=/tmp/runtime-dir \
  -e ROS2_WS=/ros2_ws \
  -e GZ_VERSION=harmonic \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /usr/lib/wsl:/usr/lib/wsl:ro \
  -v "${ROOT}:/ros2_ws" \
  "${OGRE_MOUNTS[@]}" \
  --entrypoint bash \
  leo_rover_humble \
  -lc 'source /opt/ros/humble/setup.bash && \
       source /ros2_ws/install/setup.bash && \
       export GZ_SIM_RESOURCE_PATH=/ros2_ws/install/leo_rover_description/share:/ros2_ws/src/husarion_gz_worlds/models && \
       mkdir -p /tmp/runtime-dir && chmod 700 /tmp/runtime-dir && \
       if [[ -f /ros2_ws/docker/patched/RenderSystem_GL3Plus.so ]]; then \
         mkdir -p /usr/local/share/leo_rover_gazebo && \
         touch /usr/local/share/leo_rover_gazebo/ogre_wsl_gpu_patched; \
       fi && \
       ros2 launch leo_rover_gazebo two_robots_gpu.launch.py world:='"${WORLD:-husarion_office}"' gui:='"${GUI:-true}"' num_robots:='"${NUM_ROBOTS:-1}"' enable_camera:='"${ENABLE_CAMERA:-true}"

echo "Leo sim started (container: leo_sim)"
echo "Logs: docker logs -f leo_sim"
