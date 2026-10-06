#!/bin/bash
# ============================================================
# run_experiment.sh — start CrazySim, run ONE experiment, then
# shut the simulator down again (even if the experiment crashes).
#
# Used on the Linux machine that runs CrazySim.
#
# Usage (from anywhere):
#   ./scripts/run_experiment.sh <experiment> [extra python args]
#
# <experiment> is the file name in src/experiments/ without
# "_experiment.py":
#   ./scripts/run_experiment.sh convoy_baseline
#   ./scripts/run_experiment.sh surprise_minimization
#   ./scripts/run_experiment.sh formation_distance --config my.yaml
#
# Paths are set with environment variables (defaults in brackets):
#   CRAZYSIM_DIR      CrazySim's crazyflie-firmware folder
#                     [<repo>/../CrazySim/crazyflie-firmware]
#   VENV              Python virtual environment to activate
#                     [<repo>/.venv — skipped if it does not exist]
#   N_DRONES          how many drones to spawn            [3]
#   SIM_STARTUP_WAIT  seconds to wait for CrazySim to boot [10]
#
# Example with your own paths:
#   CRAZYSIM_DIR=~/crazyflie/CrazySim/crazyflie-firmware \
#   VENV=~/crazyflie/crazysim-venv \
#   ./scripts/run_experiment.sh convoy_baseline
# ============================================================

set -e   # stop at the first command that fails


# ============================================================
# ARGUMENTS
# ============================================================

if [ -z "$1" ]; then
    echo "Usage: $0 <experiment> [extra python args]"
    echo "e.g.   $0 convoy_baseline"
    exit 1
fi

EXPERIMENT="$1"
shift              # everything after the name is passed to Python


# ============================================================
# PATHS (environment variable, or a default)
# ============================================================

# The repo root = the folder above this script, wherever it is cloned.
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# ${NAME:-default} means: use $NAME if it is set, otherwise the default.
CRAZYSIM_DIR="${CRAZYSIM_DIR:-$PROJECT_DIR/../CrazySim/crazyflie-firmware}"
VENV="${VENV:-$PROJECT_DIR/.venv}"
N_DRONES="${N_DRONES:-3}"
SIM_STARTUP_WAIT="${SIM_STARTUP_WAIT:-10}"

LAUNCH_SCRIPT="$CRAZYSIM_DIR/tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh"
EXPERIMENT_FILE="$PROJECT_DIR/src/experiments/${EXPERIMENT}_experiment.py"


# ============================================================
# CHECK EVERYTHING EXISTS BEFORE STARTING THE SIMULATOR
# ============================================================

if [ ! -f "$EXPERIMENT_FILE" ]; then
    echo "No such experiment: $EXPERIMENT_FILE"
    echo "Available:"
    ls "$PROJECT_DIR/src/experiments/" | grep "_experiment.py" | sed 's/_experiment.py//'
    exit 1
fi

if [ ! -f "$LAUNCH_SCRIPT" ]; then
    echo "CrazySim launch script not found: $LAUNCH_SCRIPT"
    echo "Set CRAZYSIM_DIR to your CrazySim crazyflie-firmware folder."
    exit 1
fi

if [ -f "$VENV/bin/activate" ]; then
    source "$VENV/bin/activate"
else
    echo "No virtual environment at $VENV — using the current Python."
fi


# ============================================================
# CLEANUP FUNCTION — stops CrazySim and Gazebo
# ============================================================

cleanup() {

    echo ""
    echo "========================================"
    echo "Stopping CrazySim..."
    echo "========================================"

    # Ask the CrazySim launcher to stop (like pressing Ctrl-C in it).
    if [ -n "$SIM_PID" ]; then
        kill -INT "$SIM_PID" 2>/dev/null || true
    fi

    sleep 2

    # Stop Gazebo and the Crazyflie SITL firmware processes.
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "gzserver" 2>/dev/null || true
    pkill -f "gzclient" 2>/dev/null || true
    pkill -f "crazyflie-simulation" 2>/dev/null || true
    pkill -f "sitl" 2>/dev/null || true

    sleep 2

    # Force-kill anything still running.
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "gzserver" 2>/dev/null || true
    pkill -9 -f "gzclient" 2>/dev/null || true

    echo "CrazySim and Gazebo stopped."
}

# Run cleanup() whenever this script exits — normally, on error, or Ctrl-C.
trap cleanup EXIT


# ============================================================
# START CRAZYSIM
# ============================================================

echo "========================================"
echo "Starting CrazySim with $N_DRONES drones"
echo "========================================"

cd "$CRAZYSIM_DIR"
bash "$LAUNCH_SCRIPT" -n "$N_DRONES" -m crazyflie &
SIM_PID=$!    # remember the launcher's process ID so cleanup() can stop it

echo "CrazySim PID: $SIM_PID"
echo "Waiting $SIM_STARTUP_WAIT s for CrazySim to start..."
sleep "$SIM_STARTUP_WAIT"


# ============================================================
# RUN THE EXPERIMENT
# ============================================================

# Run from the repo root so config/ and data/ paths resolve.
cd "$PROJECT_DIR"

echo "========================================"
echo "Running experiment: $EXPERIMENT"
echo "========================================"

# "$@" passes on any extra arguments, e.g. --config or --mode.
python3 -m "src.experiments.${EXPERIMENT}_experiment" "$@"

echo "========================================"
echo "Experiment finished — simulator will now stop."
echo "========================================"

# cleanup() runs automatically here because of the EXIT trap.
