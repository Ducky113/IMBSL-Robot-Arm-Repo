#!/usr/bin/env bash
# Gamepad teleop for SO-101 via gym_manipulator + IK.
# Requires: conda env with lerobot installed (gamepad, kinematics, feetech extras).
#
# Usage:
#   ./run_so101_teleop.sh              # teleop test (no recording)
#   ./run_so101_teleop.sh record       # record demonstrations

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
LEROBOT_DIR="${ROOT}/lerobot"
CONFIG_DIR="${ROOT}/config/robot"

MODE="${1:-teleop}"
if [[ "${MODE}" == "record" ]]; then
  CONFIG="${CONFIG_DIR}/env_config_so101_record.json"
else
  CONFIG="${CONFIG_DIR}/env_config_so101_gamepad.json"
fi

# cmeel native libs (urdfdom, pinocchio) for placo IK
CMEL_LIB="$(python -c "import pathlib; print(pathlib.Path(__import__('cmeel').__file__).resolve().parent / 'cmeel.prefix' / 'lib')" 2>/dev/null || true)"
if [[ -n "${CMEL_LIB}" && -d "${CMEL_LIB}" ]]; then
  export LD_LIBRARY_PATH="${CMEL_LIB}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

python -c "import placo" 2>/dev/null || {
  echo "placo failed to import. Fix with:"
  echo "  pip install 'placo>=0.9.16'"
  exit 1
}

cd "${LEROBOT_DIR}"
exec python -m lerobot.rl.gym_manipulator --config_path "${CONFIG}"
