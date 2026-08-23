#!/usr/bin/env bash
# Build patched Ogre RenderSystem_GL3Plus for WSLg D3D12/NVIDIA GPU.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${ROOT}/docker/patched"

mkdir -p "${OUT_DIR}"

echo "Building patched Ogre RenderSystem_GL3Plus (several minutes)..."

docker run --rm \
  -v "${ROOT}/docker/ogre-wsl-gpu.patch:/tmp/ogre-wsl-gpu.patch:ro" \
  -v "${OUT_DIR}:/out" \
  --entrypoint bash \
  leo_rover_humble \
  -lc '
    set -eo pipefail
    export DEBIAN_FRONTEND=noninteractive
    apt-get update -qq
    apt-get install -y -qq devscripts debhelper quilt >/dev/null
    sed -i "s/^# deb-src/deb-src/" /etc/apt/sources.list
    apt-get update -qq
    apt-get build-dep -y -qq ogre-next
    cd /tmp
    rm -rf ogre-next-2.2.5+dfsg3
    apt-get source -y ogre-next
    cd ogre-next-2.2.5+dfsg3
    patch -p1 < /tmp/ogre-wsl-gpu.patch
    debian/rules build >/tmp/ogre-configure.log 2>&1
    cd obj-x86_64-linux-gnu
    make RenderSystem_GL3Plus -j"$(nproc)" >/tmp/ogre-build.log 2>&1
    RS_DIR=$(dirname "$(find . -name RenderSystem_GL3Plus.so.2.2.5 | head -1)")
    cp "${RS_DIR}"/RenderSystem_GL3Plus.so* /out/
    ls -la /out/
    echo "Patched RenderSystem_GL3Plus.so ready."
  '

echo "Done. Patched libs in ${OUT_DIR}"
