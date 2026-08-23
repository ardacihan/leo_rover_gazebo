#!/usr/bin/env bash
export LD_LIBRARY_PATH=/usr/lib/wsl/lib
echo "=== nvidia-smi ==="
nvidia-smi -L 2>&1 | head -3
echo "=== WSL GL/EGL libs present ==="
ls /usr/lib/wsl/lib/ 2>/dev/null | grep -iE 'libGL|libEGL|libd3d|nvidia|libGLX' | head
echo "=== ign rendering plugins ==="
ls /usr/lib/x86_64-linux-gnu/ign-rendering*/engine-plugins/ 2>/dev/null | head
ls /opt/ros/humble/lib/ign-rendering*/engine-plugins/ 2>/dev/null | head
