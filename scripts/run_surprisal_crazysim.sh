
#!/bin/bash
set -Eeuo pipefail

# Project and CrazySim configuration
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export CRAZYSIM_DIR="${CRAZYSIM_DIR:-$HOME/CrazySim/crazyflie-firmware}"
export VENV="${VENV:-$HOME/crazysim-venv}"
export N_DRONES=2
export SIM_STARTUP_WAIT="${SIM_STARTUP_WAIT:-15}"

LAUNCH_SCRIPT="$CRAZYSIM_DIR/tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh"
SIM_PID=""

cleanup() {
    echo
    echo "Stopping CrazySim..."

    if [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; then
        kill -INT "$SIM_PID" 2>/dev/null || true
        wait "$SIM_PID" 2>/dev/null || true
    fi

    echo "Launcher process stopped."
    echo "Check for remaining Gazebo/SITL processes if needed."
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ ! -f "$LAUNCH_SCRIPT" ]]; then
    echo "ERROR: CrazySim launch script not found:"
    echo "$LAUNCH_SCRIPT"
    exit 1
fi

if [[ ! -f "$VENV/bin/activate" ]]; then
    echo "ERROR: Python environment not found: $VENV"
    exit 1
fi

source "$VENV/bin/activate"

cd "$CRAZYSIM_DIR"
echo "Starting CrazySim with $N_DRONES drones..."

bash "$LAUNCH_SCRIPT" -n "$N_DRONES" -m crazyflie &
SIM_PID=$!

echo "Waiting ${SIM_STARTUP_WAIT}s for simulator startup..."
sleep "$SIM_STARTUP_WAIT"

if ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "ERROR: CrazySim launcher exited during startup."
    exit 1
fi

cd "$PROJECT_DIR"

echo "Running surprisal distance experiment in SIMULATION mode..."
python -m src.tests.surprisal_distance_crazysim --mode sim

echo "Experiment finished."
