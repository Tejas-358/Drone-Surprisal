#!/bin/bash

set -e

# ============================================================
# CONFIGURATION
# ============================================================

CRAZYFLIE_DIR="$HOME/crazyflie/CrazySim/crazyflie-firmware"
VENV="$HOME/crazyflie/crazysim-venv"
PROJECT_DIR="$HOME/crazyflie/three_cf_project"

# Python experiment to run
EXPERIMENT="surprise_minimization_experiment.py"


# ============================================================
# ACTIVATE VIRTUAL ENVIRONMENT
# ============================================================

source "$VENV/bin/activate"


# ============================================================
# CLEANUP FUNCTION
# ============================================================

cleanup() {

    echo ""
    echo "========================================"
    echo "Stopping CrazySim..."
    echo "========================================"

    # --------------------------------------------------------
    # Stop the CrazySim launcher
    # --------------------------------------------------------

    if [ -n "$SIM_PID" ]; then
        kill -INT "$SIM_PID" 2>/dev/null || true
    fi

    sleep 2


    # --------------------------------------------------------
    # Kill Gazebo processes
    # --------------------------------------------------------

    echo "Stopping Gazebo..."

    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "gzserver" 2>/dev/null || true
    pkill -f "gzclient" 2>/dev/null || true


    # --------------------------------------------------------
    # Kill Crazyflie SITL processes
    # --------------------------------------------------------

    echo "Stopping Crazyflie SITL..."

    pkill -f "crazyflie-simulation" 2>/dev/null || true
    pkill -f "sitl" 2>/dev/null || true


    # --------------------------------------------------------
    # Give processes time to exit
    # --------------------------------------------------------

    sleep 2


    # --------------------------------------------------------
    # Force kill anything still remaining
    # --------------------------------------------------------

    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "gzserver" 2>/dev/null || true
    pkill -9 -f "gzclient" 2>/dev/null || true


    echo ""
    echo "CrazySim and Gazebo stopped."
    echo ""
}


# ============================================================
# ALWAYS CLEAN UP WHEN SCRIPT EXITS
# ============================================================

trap cleanup EXIT


# ============================================================
# START CRAZYSIM
# ============================================================

echo ""
echo "========================================"
echo "Starting CrazySim with 3 drones"
echo "========================================"
echo ""

cd "$CRAZYFLIE_DIR"

bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh \
    -n 3 \
    -m crazyflie &

SIM_PID=$!

echo "CrazySim PID: $SIM_PID"


# ============================================================
# WAIT FOR SIMULATION
# ============================================================

echo ""
echo "Waiting for CrazySim to start..."

sleep 10

echo "CrazySim should now be ready."


# ============================================================
# RUN EXPERIMENT
# ============================================================

cd "$PROJECT_DIR"

echo ""
echo "========================================"
echo "Starting experiment"
echo "========================================"
echo ""

python3 "$EXPERIMENT"


# ============================================================
# EXPERIMENT FINISHED
# ============================================================

echo ""
echo "========================================"
echo "Experiment finished"
echo "========================================"

echo ""
echo "Simulation will now be stopped."


# cleanup() is automatically called by the EXIT trap