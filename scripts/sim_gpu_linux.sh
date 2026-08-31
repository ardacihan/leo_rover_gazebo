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

CONTAINER_NAME="${CONTAINER_NAME:-leo_sim}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
IGN_PARTITION="${IGN_PARTITION:-leo_${CONTAINER_NAME}}"
GZ_PARTITION="${GZ_PARTITION:-$IGN_PARTITION}"

docker stop "$CONTAINER_NAME" 2>/dev/null || true
docker rm "$CONTAINER_NAME" 2>/dev/null || true

# Headless must present no display at all: with DISPLAY set, Ogre takes the
# GLX/X11 path and can land on llvmpipe.
SIM_DISPLAY="${DISPLAY:-:0}"
X11_MOUNT=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
if [[ "${GUI:-true}" != "true" ]]; then
  SIM_DISPLAY=""
  X11_MOUNT=()
fi

# Launch args are passed as container env so the inner script needs no
# host-side quote interpolation (the previous -lc '...' form was easy to break).
docker run -d --name "$CONTAINER_NAME" \
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
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  -e IGN_PARTITION="$IGN_PARTITION" \
  -e GZ_PARTITION="$GZ_PARTITION" \
  -e LEO_WORLD="${WORLD:-husarion_office}" \
  -e LEO_GUI="${GUI:-true}" \
  -e LEO_NUM_ROBOTS="${NUM_ROBOTS:-1}" \
  -e LEO_ENABLE_CAMERA="${ENABLE_CAMERA:-true}" \
  -e LEO_GT_ODOM_TF="${GT_ODOM_TF:-true}" \
  -e LEO_SIM_SPEED="${SIM_SPEED:-1.0}" \
  "${X11_MOUNT[@]}" \
  -v "${ROOT}:/ros2_ws" \
  -v "${ROOT}/scripts/10_nvidia.json:/usr/share/glvnd/egl_vendor.d/10_nvidia.json:ro" \
  --entrypoint bash \
  "${LEO_IMAGE:-leo_rover_humble:bundle}" \
  -lc "source /opt/ros/humble/setup.bash && \
       source /ros2_ws/install/setup.bash && \
       export GZ_SIM_RESOURCE_PATH=/ros2_ws/install/leo_rover_description/share:/ros2_ws/src/husarion_gz_worlds/models:/ros2_ws/src/leo_rover_gazebo/models:/ros2_ws/src/aws_small_house/models && \
       mkdir -p /tmp/runtime-dir && chmod 700 /tmp/runtime-dir && \
       ros2 launch leo_rover_gazebo two_robots_gpu.launch.py \
         world:=\${LEO_WORLD} gui:=\${LEO_GUI} num_robots:=\${LEO_NUM_ROBOTS} \
         enable_camera:=\${LEO_ENABLE_CAMERA} gt_odom_tf:=\${LEO_GT_ODOM_TF} \
         sim_speed:=\${LEO_SIM_SPEED}"

echo "Leo sim started (container: $CONTAINER_NAME, domain=$ROS_DOMAIN_ID, partition=$IGN_PARTITION)"
echo "Logs: docker logs -f $CONTAINER_NAME"
