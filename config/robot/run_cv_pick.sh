#!/usr/bin/env bash
# Detect a box with the CV service and move the SO-101 to pick it.
#
# Usage:
#   ./run_cv_pick.sh --image ../data/images/M1.jpg        # dry run (print target)
#   ./run_cv_pick.sh --camera 0 --go --pick               # live: hover then pick
#   ./run_cv_pick.sh --calibrate points.json              # solve CV->robot transform
#
# All flags are forwarded to mission/cv_pick.py.

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
MISSION_DIR="${ROOT}/mission"

# cmeel native libs (urdfdom, pinocchio) for placo IK
CMEL_LIB="$(python -c "import pathlib; print(pathlib.Path(__import__('cmeel').__file__).resolve().parent / 'cmeel.prefix' / 'lib')" 2>/dev/null || true)"
if [[ -n "${CMEL_LIB}" && -d "${CMEL_LIB}" ]]; then
  export LD_LIBRARY_PATH="${CMEL_LIB}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

python -c "import placo" 2>/dev/null || {
  echo "placo failed to import. Fix with: pip install 'placo>=0.9.16'"
  exit 1
}

export PYTHONPATH="${ROOT}/lerobot/src:${ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
cd "${MISSION_DIR}"
exec python cv_pick.py "$@"
