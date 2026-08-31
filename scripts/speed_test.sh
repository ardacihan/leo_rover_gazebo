#!/usr/bin/env bash
# Bring up the 2-robot sim (cameras off, GPU) and measure real-time factor +
# GPU utilization. No SLAM/Nav - this is the raw sim ceiling.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -e /dev/dxg || -d /usr/lib/wsl ]]; then
  SPEED_LAUNCHER="$ROOT/scripts/sim_gpu_wsl.sh"
else
  SPEED_LAUNCHER="$ROOT/scripts/sim_gpu_linux.sh"
fi
WORLD="${WORLD:-office_world}" GUI=false NUM_ROBOTS=2 ENABLE_CAMERA=false \
  bash "$SPEED_LAUNCHER"

in_sim() { docker exec leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }

echo "[speed] waiting for /leo2/scan"
for i in $(seq 1 40); do
  if in_sim 'ros2 topic list 2>/dev/null | grep -q "^/leo2/scan$"'; then break; fi
  sleep 3
done
sleep 5

echo "[speed] measuring RTF over 15 s wall..."
read s1 <<< "$(in_sim 'ros2 topic echo /clock --once 2>/dev/null | grep -m1 "sec:" | grep -oE "[0-9]+"')"
sleep 15
read s2 <<< "$(in_sim 'ros2 topic echo /clock --once 2>/dev/null | grep -m1 "sec:" | grep -oE "[0-9]+"')"
adv=$(( s2 - s1 ))
echo "[speed] sim advanced ${adv}s in 15s wall  ->  RTF ~ $(awk "BEGIN{printf \"%.2f\", $adv/15}")"

echo "[speed] GPU utilization now:"
docker exec leo_sim bash -lc 'LD_LIBRARY_PATH=/usr/lib/wsl/lib nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader 2>/dev/null' || echo '  (nvidia-smi unavailable)'
echo "[speed] renderer check (ign server log):"
docker logs leo_sim 2>&1 | grep -iE 'render|ogre|llvmpipe|d3d12|GL_' | head -4 || true

docker stop leo_sim >/dev/null
echo "[speed] done"
