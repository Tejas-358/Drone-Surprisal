#!/bin/bash
# ============================================================
# run_all_sim.sh — run EVERY experiment in CrazySim, several times each,
# then run all analysis scripts. Meant for the Linux server.
#
# Each run starts a fresh CrazySim (via run_experiment.sh) and stops it
# afterwards, so runs cannot affect each other. A failed run is reported
# and skipped; the batch continues.
#
# Usage (from the repo root):
#   ./scripts/run_all_sim.sh                 # everything, 3 repeats each
#   REPEATS=1 ./scripts/run_all_sim.sh       # quick check: 1 run each
#   ONLY=target_swarm ./scripts/run_all_sim.sh   # one experiment only
#
# Settings (environment variables, defaults in brackets):
#   REPEATS       runs per condition                         [3]
#   NOISE         position/velocity noise std in m, m/s,
#                 to mimic real Lighthouse jitter             [0.03]
#   SEED_BASE     first random seed; run r uses SEED_BASE+r   [100]
#   N_SWEEP       convoy E1 surprise sensitivities N to test  ["0.05 0.1 0.2 0.4"]
#   ONLY          goal_navigation | target_swarm | convoy | formation   [all]
#   plus everything run_experiment.sh uses: CRAZYSIM_DIR, VENV, SIM_STARTUP_WAIT
#
# Total time (default): about 40 runs x ~1.5 min (sim start + flight) = ~1 h.
# ============================================================

REPEATS="${REPEATS:-3}"
NOISE="${NOISE:-0.03}"
SEED_BASE="${SEED_BASE:-100}"
N_SWEEP="${N_SWEEP:-0.05 0.1 0.2 0.4}"
ONLY="${ONLY:-all}"

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
RUN="$PROJECT_DIR/scripts/run_experiment.sh"
FAILED=()

# run_one <n_drones> <experiment> [python args...]
# Runs one experiment once; remembers it if it failed.
run_one() {
    local drones="$1"; shift
    echo ""
    echo "################################################################"
    echo "# $* (drones: $drones)"
    echo "################################################################"
    if ! N_DRONES="$drones" "$RUN" "$@" --mode sim; then
        FAILED+=("$*")
    fi
}

wants() { [ "$ONLY" = "all" ] || [ "$ONLY" = "$1" ]; }

for r in $(seq 1 "$REPEATS"); do
    SEED=$((SEED_BASE + r))
    COMMON=(--noise "$NOISE" --seed "$SEED")

    # --- Two-drone surprisal distance: 2 controllers x 2 scenarios
    if wants goal_navigation; then
        for controller in fixed adaptive; do
            for scenario in too_far too_close; do
                run_one 2 surprisal_goal_navigation --controller "$controller" \
                        --scenario "$scenario" "${COMMON[@]}"
            done
        done
    fi

    # --- Three drones to a target: baseline vs surprise
    if wants target_swarm; then
        for controller in goal_only surprise; do
            run_one 3 target_swarm --controller "$controller" "${COMMON[@]}"
        done
    fi

    # --- Convoy: E2 baseline once, E1 surprise for every N in the sweep
    if wants convoy; then
        run_one 3 convoy_baseline "${COMMON[@]}"
        for N in $N_SWEEP; do
            run_one 3 surprise_minimization --set "noise_scale=$N" "${COMMON[@]}"
        done
    fi

    # --- E3 formation distance
    if wants formation; then
        run_one 3 formation_distance "${COMMON[@]}"
    fi
done

# --- Analysis of everything collected so far (sim runs)
cd "$PROJECT_DIR"
if [ -f "${VENV:-$HOME/crazysim-venv}/bin/activate" ]; then
    source "${VENV:-$HOME/crazysim-venv}/bin/activate"
fi
echo ""
echo "================================================================"
echo "ANALYSIS"
echo "================================================================"
python3 -m analysis.plot_goal_navigation_runs --mode sim
python3 -m analysis.plot_target_swarm_runs --mode sim
python3 -m analysis.plot_convoy_runs --mode sim
python3 -m analysis.plot_formation_runs --mode sim
echo "Figures are in analysis/figures/"

if [ ${#FAILED[@]} -gt 0 ]; then
    echo ""
    echo "These runs FAILED (see their output above):"
    printf '  %s\n' "${FAILED[@]}"
    exit 1
fi
echo "All runs finished."
