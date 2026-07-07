#!/usr/bin/env bash
# Move SO-101 to table-frame (x, y, z). Origin at robot base, z=0 = table.
#
# Usage:
#   ./run_goto_table.sh --x 0.2 --y 0.0
#   ./run_goto_table.sh where
#   ./run_goto_table.sh --interactive

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
MISSION_DIR="${ROOT}/mission"

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
exec python goto_table.py "$@"
