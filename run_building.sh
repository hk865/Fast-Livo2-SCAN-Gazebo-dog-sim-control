#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "$(docker ps --filter name=^/fastlivo2-building-navigation$ --format '{{.Names}}')" ]]; then
  echo 'An owned Building navigation container is already running' >&2; exit 2
fi
docker run --rm --name fastlivo2-building-navigation --gpus all --network=none \
  --cpus=12 --memory=12g --shm-size=2g --user "$(id -u):$(id -g)" \
  -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute -e EGL_PLATFORM=surfaceless \
  --mount "type=bind,src=$project_dir,dst=/workspace/navigation" \
  --entrypoint /usr/bin/python3 "${NAVIGATION_IMAGE:-go2-navigation:local}" \
  -B /workspace/navigation/start.py "$@"
