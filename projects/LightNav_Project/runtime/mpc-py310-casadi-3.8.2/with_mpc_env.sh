#!/bin/bash
set -eo pipefail
runtime_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
[[ -d "$runtime_root/site-packages/casadi" ]] || { echo 'Runtime is not installed' >&2; exit 2; }
source /opt/ros/humble/setup.bash
source /home/nvidia/LightNav_Project/robot_deploy/install/local_setup.bash
export PYTHONPATH="$runtime_root/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
# Deliberately do not choose domain or run a node; caller supplies explicit command.
[[ $# -gt 0 ]] || { echo 'Usage: bash with_mpc_env.sh <command> [args...]' >&2; exit 2; }
exec "$@"
