#!/usr/bin/env bash
# One-shot setup for the NCKU SO-101 pick-and-place project.
#
# Run this once after cloning the repo from GitHub:
#
#   git clone <repo-url>
#   cd <repo>/Main        # the directory containing this script
#   ./setup.sh
#
# It creates a Python environment, installs the vendored LeRobot (with the
# SO-101 gamepad patches already applied) plus the extras this project needs,
# and verifies the install.
#
# Environment overrides (optional):
#   LEROBOT_ENV_NAME       conda env name         (default: lerobot)
#   LEROBOT_PYTHON_VERSION python version         (default: 3.11)
#   USE_VENV=1             force a venv instead of conda

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEROBOT_DIR="${ROOT}/lerobot"
ENV_NAME="${LEROBOT_ENV_NAME:-lerobot}"
PYTHON_VERSION="${LEROBOT_PYTHON_VERSION:-3.11}"

info()  { printf '\033[1;34m==>\033[0m %s\n' "$*"; }
warn()  { printf '\033[1;33mWARN:\033[0m %s\n' "$*"; }
die()   { printf '\033[1;31mERROR:\033[0m %s\n' "$*" >&2; exit 1; }

[[ -d "${LEROBOT_DIR}" ]] || die "lerobot/ not found at ${LEROBOT_DIR}. Run this from the project's Main/ directory."

# ---------------------------------------------------------------------------
# 1. Python environment (conda preferred, venv fallback)
# ---------------------------------------------------------------------------
if [[ "${USE_VENV:-0}" != "1" ]] && command -v conda >/dev/null 2>&1; then
  info "Using conda (env: ${ENV_NAME}, python ${PYTHON_VERSION})"
  # shellcheck disable=SC1091
  source "$(conda info --base)/etc/profile.d/conda.sh"
  if ! conda env list | awk '{print $1}' | grep -qx "${ENV_NAME}"; then
    info "Creating conda env '${ENV_NAME}'"
    conda create -y -n "${ENV_NAME}" "python=${PYTHON_VERSION}"
  else
    info "Conda env '${ENV_NAME}' already exists — reusing it"
  fi
  conda activate "${ENV_NAME}"
else
  info "Using a local virtualenv at ${ROOT}/.venv"
  command -v python3 >/dev/null 2>&1 || die "python3 not found. Install Python ${PYTHON_VERSION}+ or conda."
  [[ -d "${ROOT}/.venv" ]] || python3 -m venv "${ROOT}/.venv"
  # shellcheck disable=SC1091
  source "${ROOT}/.venv/bin/activate"
fi

# ---------------------------------------------------------------------------
# 2. Install LeRobot (vendored, editable) + project extras
# ---------------------------------------------------------------------------
info "Upgrading pip"
python -m pip install --upgrade pip

info "Installing vendored LeRobot with [gamepad,kinematics,feetech] extras"
python -m pip install -e "${LEROBOT_DIR}[gamepad,kinematics,feetech]"

# The kinematics extra pins placo <0.9.16, but this project needs >=0.9.16
# for the SO-101 IK. Install it last so it wins.
info "Installing placo >= 0.9.16 (overrides the LeRobot pin)"
python -m pip install 'placo>=0.9.16'

# ---------------------------------------------------------------------------
# 3. Verify
# ---------------------------------------------------------------------------
info "Verifying imports"

# placo's native libs (urdfdom, pinocchio) ship inside the cmeel package.
CMEL_LIB="$(python -c "import pathlib, cmeel; print(pathlib.Path(cmeel.__file__).resolve().parent / 'cmeel.prefix' / 'lib')" 2>/dev/null || true)"
if [[ -n "${CMEL_LIB}" && -d "${CMEL_LIB}" ]]; then
  export LD_LIBRARY_PATH="${CMEL_LIB}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

python - <<PY
import importlib, sys
ok = True
for mod in ("lerobot", "cv2", "numpy", "placo"):
    try:
        m = importlib.import_module(mod)
        print(f"  {mod:8s} {getattr(m, '__version__', 'ok')}")
    except Exception as e:  # noqa: BLE001
        ok = False
        print(f"  {mod:8s} FAILED: {e}")

# Confirm the trimmed URDF still loads its meshes.
try:
    import placo
    urdf = "${ROOT}/assets/SO-ARM100/Simulation/SO101/so101_new_calib.urdf"
    placo.RobotWrapper(urdf)
    print("  URDF     loads OK")
except Exception as e:  # noqa: BLE001
    ok = False
    print(f"  URDF     FAILED: {e}")

sys.exit(0 if ok else 1)
PY

info "Setup complete."
cat <<EOF

Next steps:
  1. Activate the environment:
       conda activate ${ENV_NAME}          # (or: source .venv/bin/activate)
  2. Plug in the SO-101 and gamepad. Check the serial port:
       ls /dev/ttyACM*
     Update "port" in config/robot/*.json if it is not /dev/ttyACM0.
  3. See docs/USAGE.md for how to run each part of the project.
EOF
