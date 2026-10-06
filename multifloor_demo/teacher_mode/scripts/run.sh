#!/usr/bin/env bash
set -e
TEACHER_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=79 ROS_LOCALHOST_ONLY=1 GZ_IP=127.0.0.1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
mode="${1:-serve}"; if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in
 build) cmake -S "$TEACHER_ROOT/simulation" -B "$TEACHER_ROOT/simulation/build" -DCMAKE_BUILD_TYPE=Release
        exec cmake --build "$TEACHER_ROOT/simulation/build" -j2 ;;
 prepare) exec python3 "$TEACHER_ROOT/simulation/prepare.py" "$@" ;;
 test) exec python3 "$TEACHER_ROOT/scripts/run_test.py" "$@" ;;
 evaluate) exec python3 "$TEACHER_ROOT/scripts/evaluate.py" "$@" ;;
 serve) exec python3 "$TEACHER_ROOT/scripts/serve.py" "$@" ;;
 *) echo 'Usage: teacher_mode/scripts/run.sh {build|prepare|test|evaluate|serve}' >&2; exit 2 ;;
esac
