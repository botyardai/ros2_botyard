#!/usr/bin/env bash
set -e

HAND_SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-}"
CHECK_ONLY=0
ISAAC_ARGS=()

usage() {
  cat <<'USAGE'
Usage: bash start_sim.sh [options] [-- Isaac Sim arguments...]
  --isaac-dir PATH  Specify the Isaac Sim 5.1 installation
  --check           Check paths without changing files or starting Isaac Sim
  -h, --help        Show this help
Starts Isaac Sim with ROS Bridge and opens right_hand.usd. Click Play to simulate.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --isaac-dir)
      if [[ $# -lt 2 || -z "$2" ]]; then
        echo '--isaac-dir requires an installation directory.' >&2
        exit 1
      fi
      ISAAC_SIM_DIR="$2"
      shift 2
      ;;
    --check) CHECK_ONLY=1; shift ;;
    -h|--help) usage; exit 0 ;;
    --) shift; ISAAC_ARGS+=("$@"); break ;;
    *) ISAAC_ARGS+=("$1"); shift ;;
  esac
done

is_isaac_install() {
  [[ -x "$1/isaac-sim.sh" && -x "$1/python.sh" &&
     -d "$1/exts/isaacsim.ros2.bridge/jazzy/lib" &&
     -f "$1/VERSION" && "$(head -n 1 "$1/VERSION")" == 5.1.* ]]
}

# Prefer an explicit path, then a previously selected path or a launcher on PATH.
if [[ -z "$ISAAC_SIM_DIR" && -f "$HAND_SIM_DIR/.cache/isaac_sim_dir" ]]; then
  IFS= read -r cached_dir < "$HAND_SIM_DIR/.cache/isaac_sim_dir"
  if is_isaac_install "$cached_dir"; then
    ISAAC_SIM_DIR="$cached_dir"
  fi
fi
if [[ -z "$ISAAC_SIM_DIR" ]]; then
  launcher_path="$(type -P isaac-sim.sh || true)"
  if [[ -n "$launcher_path" ]]; then
    launcher_dir="$(dirname "$(readlink -f "$launcher_path")")"
    if is_isaac_install "$launcher_dir"; then
      ISAAC_SIM_DIR="$launcher_dir"
    fi
  fi
fi
if [[ -z "$ISAAC_SIM_DIR" ]]; then
  candidates=()
  shopt -s nullglob
  for candidate in \
    "$HOME"/isaacsim* "$HOME"/isaac-sim* \
    "$HOME"/Downloads/isaac* "$HOME"/下载/isaac* \
    "$HOME"/.local/share/ov/pkg/isaac* \
    "$HOME"/apps/isaac* "$HOME"/Applications/isaac* \
    /opt/isaac* "$HAND_SIM_DIR"/../isaac* "$HAND_SIM_DIR"/../../isaac*; do
    if is_isaac_install "$candidate"; then
      candidate="$(cd "$candidate" && pwd -P)"
      duplicate=0
      for existing in "${candidates[@]}"; do
        [[ "$existing" != "$candidate" ]] || duplicate=1
      done
      [[ "$duplicate" == 1 ]] || candidates+=("$candidate")
    fi
  done
  shopt -u nullglob
  if [[ ${#candidates[@]} == 1 ]]; then
    ISAAC_SIM_DIR="${candidates[0]}"
  elif [[ ${#candidates[@]} -gt 1 ]]; then
    echo 'Found multiple Isaac Sim installations. Choose one with --isaac-dir:' >&2
    printf '  %s\n' "${candidates[@]}" >&2
    exit 1
  fi
fi
if [[ -z "$ISAAC_SIM_DIR" ]]; then
  echo 'Isaac Sim was not found. Install Isaac Sim 5.1, then run:' >&2
  echo '  bash start_sim.sh --isaac-dir /path/to/isaacsim' >&2
  exit 1
fi
if ! is_isaac_install "$ISAAC_SIM_DIR"; then
  echo "Invalid Isaac Sim installation: $ISAAC_SIM_DIR (requires Isaac Sim 5.1 with isaac-sim.sh, python.sh and the Jazzy bridge)." >&2
  exit 1
fi
ISAAC_SIM_DIR="$(cd "$ISAAC_SIM_DIR" && pwd -P)"
for required_file in "$HAND_SIM_DIR/right_hand.usd" "$HAND_SIM_DIR/load_scene.py"; do
  if [[ ! -f "$required_file" ]]; then
    echo "Project download is incomplete; missing: $required_file" >&2
    exit 1
  fi
done

echo "Isaac Sim: $ISAAC_SIM_DIR"
echo "Scene: $HAND_SIM_DIR/right_hand.usd"
if [[ "$CHECK_ONLY" == 1 ]]; then
  echo 'Single-hand launcher: ready.'
  exit 0
fi

unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION RMW_IMPLEMENTATION
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-14}"
export PYTHONNOUSERSITE=1
export LD_LIBRARY_PATH="$ISAAC_SIM_DIR/exts/isaacsim.ros2.bridge/jazzy/lib"

mkdir -p "$HAND_SIM_DIR/.cache"
printf '%s\n' "$ISAAC_SIM_DIR" > "$HAND_SIM_DIR/.cache/isaac_sim_dir"
cd "$HAND_SIM_DIR"
exec "$ISAAC_SIM_DIR/isaac-sim.sh" \
  --enable isaacsim.ros2.bridge \
  --/isaac/startup/create_new_stage=false \
  --exec ./load_scene.py "${ISAAC_ARGS[@]}"
