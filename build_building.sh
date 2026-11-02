#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
image="${NAVIGATION_IMAGE:-go2-navigation:local}"
if [[ -n "$(docker ps --filter name=^/fastlivo2-building-navigation$ --format '{{.Names}}')" ]]; then
  echo 'Finish the active Building run before rebuilding' >&2; exit 2
fi
# Dependency prefix comes from the existing environment image, never the old host runtime.
if [[ ! -e "$project_dir/install" ]]; then
  docker run --rm --network=none --user "$(id -u):$(id -g)" \
    --mount "type=bind,src=$project_dir,dst=/project" --entrypoint /bin/bash "$image" \
    -c 'cp -a /workspace/navigation/install /project/install'
fi
docker run --rm --name fastlivo2-building-build --network=none --cpus=8 --memory=8g \
  --user "$(id -u):$(id -g)" --mount "type=bind,src=$project_dir,dst=/workspace/navigation" \
  --entrypoint /bin/bash "$image" -c '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /workspace/navigation/install/ros/local_setup.bash
    cd /workspace/navigation
    mkdir -p build/pct_routes install/pct
    g++ -std=c++17 -O2 -I/usr/include/eigen3 -Isrc/planning/pct/native \
      src/planning/pct/pct_cli.cpp src/planning/pct/native/a_star/a_star_search.cc -o install/pct/pct_astar
    cmake -S src/planning/scan/plan_manage -B build/building_scan -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX=/workspace/navigation/install/ros
    cmake --build build/building_scan --target scan_planner_node -j6
    cp build/building_scan/scan_planner_node install/ros/lib/scan_planner/scan_planner_node
    cmake -S src/localization/frontend/fast_livo2_core -B build/backend_core -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX=/workspace/navigation/install/ros
    cmake --build build/backend_core --target fast_livo2_core -j2
    cmake --install build/backend_core
    cmake -S src/localization/backend/native -B build/backend_graph -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/workspace/navigation/install/backend
    cmake --build build/backend_graph -j2
    cmake --install build/backend_graph
    OPENBLAS_NUM_THREADS=1 python3 -B src/planning/pct/generate_routes.py
    for case_name in stairs_short single_floor_turn combined_floor1_inside multi_floor2; do
      cp "build/pct_routes/${case_name}_route.json" "assets/building/routes/${case_name}.json"
    done
    python3 -B tools/validation/check_building_sources.py
    python3 -B start.py --check
  '
