"""
convoy_baseline_experiment.py — E2: the leader–follower convoy WITHOUT
surprise gating (the baseline that E1 is compared against).

What happens in one run:
  1. Connect to 3 drones, pre-flight check, take off (done by fly_safely).
  2. Fly to the start line along x (CF1 in front), each drone at its own
     height on the way so paths cannot collide.
  3. Control loop: CF1 flies towards a virtual wall and stops in front of
     it. CF2 follows CF1, CF3 follows CF2, using plain P-control (plus the
     safety repulsion) from src/controllers/convoy_controller.py.
     Every step, safety.check_in_flight() aborts if anything is unsafe.
  4. Land (fly_safely), save CSV + JSON (also after an abort) with metrics.

The control loop lives in run_convoy() so that E1
(surprise_minimization_experiment.py) reuses it with use_surprise=True.
E1 and E2 therefore fly exactly the same way except for the gating.

Run from the repo root:
    sim:   python -m src.experiments.convoy_baseline_experiment --mode sim
    real:  python -m src.experiments.convoy_baseline_experiment --mode real --config config/experiments/convoy_real.yaml
    extra: --noise 0.03 --seed 1      (sim: mimic Lighthouse jitter)
           --set noise_scale=0.1      (change one config value, e.g. for a sweep)
"""

import warnings

warnings.filterwarnings(
    "ignore",
    message="Using legacy TYPE_VELOCITY_WORLD_LEGACY.*",
    category=DeprecationWarning,
)

import time

from src.utils import cli
from src.utils.flight import fly_safely, fly_to_positions
from src.utils.observation import Observer
from src.utils.run_log import save_run
from src.utils.safety import check_in_flight
from src.controllers.convoy_controller import leader_command, follower_command, clamp


DEFAULT_CONFIG = "config/experiments/convoy.yaml"

# Gentle correction that keeps each drone on its lane (y = 0) and at its
# height while it moves along x. Without it, velocity commands alone let
# y and z slowly drift on a real drone.
HOLD_GAIN = 1.0        # m/s of correction per metre of error
HOLD_MAX_SPEED = 0.2   # m/s, limit on that correction


def send_velocity(scf, vx, state, config):
    """
    Send a forward (x) velocity command to one drone, while holding y = 0
    and z = flight height.

    Input:  scf    — one connected drone
            vx     — velocity along x in m/s (world frame), from the controller
            state  — this drone's latest telemetry (uses y, z)
            config — uses height
    Output: none.

    Called from: run_convoy(), every control step for each drone that is moving.
    """
    vy = clamp(HOLD_GAIN * (0.0 - state["y"]), -HOLD_MAX_SPEED, HOLD_MAX_SPEED)
    vz = clamp(HOLD_GAIN * (config["height"] - state["z"]), -HOLD_MAX_SPEED, HOLD_MAX_SPEED)
    scf.cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0.0)


def fly_to_start_line(drones, states, config):
    """
    Move the hovering drones to their start x positions (y = 0), each at
    its own height on the way (see flight.fly_to_positions).

    Called from: run_convoy() here and formation_distance_experiment.py.
    """
    fly_to_positions(drones, states, config, [(x, 0.0) for x in config["start_x"]])


def run_convoy(drones, config, states, use_surprise, rows, observer):
    """
    The control loop shared by E1 and E2.

    Input:  drones       — list of 3 connected, hovering drones (from fly_safely)
            config       — default.yaml + convoy settings
            states       — live telemetry from fly_safely (states[0] = CF1)
            use_surprise — False for E2 (baseline), True for E1 (gated)
            rows         — an empty list; one dictionary per control step is
                           appended (passed in so the data survives an abort)
            observer     — adds the optional --noise to what the controller sees
    Output: none.

    Called from: fly_safely() (as the 'mission'), via run_experiment().
    """
    try:
        fly_to_start_line(drones, states, config)

        print("Starting convoy control loop...")
        leader_stopped = False
        leader_hold_x = None   # where the leader parks once it has stopped
        start_time = time.time()

        while True:
            step_start = time.time()
            t = step_start - start_time
            if t >= config["duration"]:
                break

            # Safety uses the TRUE state: arena, fly-away, stale data, and
            # (optional) minimum gap.
            check_in_flight(states, config, config.get("min_gap_abort"))

            # What the controller sees (with --noise, a noisy copy).
            seen = [observer.see(s) for s in states]
            x = [s["x"] for s in seen]
            v = [s["vx"] for s in seen]

            # --- Leader (CF1)
            leader_vx, leader_stopped = leader_command(x[0], leader_stopped, config)

            # --- Followers: CF2 follows CF1, CF3 follows CF2
            cf2 = follower_command(x[0], x[1], v[0], v[1], config, use_surprise)
            cf3 = follower_command(x[1], x[2], v[1], v[2], config, use_surprise)

            # --- Send all three commands
            if leader_stopped:
                # "Velocity 0" alone lets the leader drift. Once stopped, park
                # it at a fixed point with a position setpoint instead.
                if leader_hold_x is None:
                    leader_hold_x = x[0]
                drones[0].cf.commander.send_position_setpoint(
                    leader_hold_x, 0.0, config["height"], 0.0)
            else:
                send_velocity(drones[0], leader_vx, states[0], config)
            send_velocity(drones[1], cf2["command"], states[1], config)
            send_velocity(drones[2], cf3["command"], states[2], config)

            # --- Record this step. Gaps use the TRUE positions (what really
            #     happened); the controller values are what it decided on.
            true_x = [s["x"] for s in states]
            rows.append({
                "t": round(t, 3),
                "x1": true_x[0], "x2": true_x[1], "x3": true_x[2],
                "v1": states[0]["vx"], "v2": states[1]["vx"], "v3": states[2]["vx"],
                "leader_stopped": leader_stopped,
                "gap_12": true_x[0] - true_x[1],
                "gap_23": true_x[1] - true_x[2],
                "S2": cf2["prediction_error"], "S3": cf3["prediction_error"],
                "U2": cf2["surprise"], "U3": cf3["surprise"],
                "factor2": cf2["speed_factor"], "factor3": cf3["speed_factor"],
                "repel2": cf2["repelling"], "repel3": cf3["repelling"],
                "cmd1": leader_vx, "cmd2": cf2["command"], "cmd3": cf3["command"],
            })

            if len(rows) % 10 == 1:   # about once per second
                print(f"t={t:5.1f}s  x=[{true_x[0]:5.2f} {true_x[1]:5.2f} {true_x[2]:5.2f}]  "
                      f"gaps=[{rows[-1]['gap_12']:.2f} {rows[-1]['gap_23']:.2f}]  "
                      f"factor=[{cf2['speed_factor']:.2f} {cf3['speed_factor']:.2f}]")

            # Sleep only for what is LEFT of the control period (steady 10 Hz).
            step_time = time.time() - step_start
            if step_time > 0.3:
                print(f"WARNING: control step took {step_time:.2f} s")
            time.sleep(max(0.0, config["control_period"] - step_time))

    finally:
        # Stop horizontal motion before fly_safely() lands the drones.
        for scf in drones:
            try:
                scf.cf.commander.send_velocity_world_setpoint(0.0, 0.0, 0.0, 0.0)
            except Exception:
                pass


def metrics(rows, config):
    """
    Per-run results for comparing E1 and E2 (all distances in m, times in s):

      min_gap              — closest any follower came to the drone ahead
      stop_time            — when the leader stopped (empty if it never did)
      overshoot_after_stop — how far a gap went BELOW follow_distance after
                             the leader stopped (how hard the convoy "bunched up")
      settling_after_stop  — time after the stop until both gaps stay within
                             gap_tolerance of follow_distance (empty if never)
      final_gap_error      — mean |gap - follow_distance| at the end
      low_factor_fraction  — share of steps with speed factor < 0.1 (both
                             followers): high = gating is just freezing the drones
      repel_fraction       — share of steps where the safety push was active
    """
    target = config["follow_distance"]
    tol = config.get("gap_tolerance", 0.08)
    gaps = [(r["gap_12"], r["gap_23"]) for r in rows]

    stop_i = next((i for i, r in enumerate(rows) if r["leader_stopped"]), None)
    result = {"min_gap": round(min(min(g) for g in gaps), 3),
              "stop_time": "", "overshoot_after_stop": "", "settling_after_stop": ""}
    if stop_i is not None:
        after = gaps[stop_i:]
        result["stop_time"] = round(rows[stop_i]["t"], 2)
        result["overshoot_after_stop"] = round(max(0.0, target - min(min(g) for g in after)), 3)
        for i in range(stop_i, len(rows)):
            if all(abs(a - target) <= tol and abs(b - target) <= tol for a, b in gaps[i:]):
                result["settling_after_stop"] = round(rows[i]["t"] - rows[stop_i]["t"], 2)
                break

    n = len(rows)
    result["final_gap_error"] = round(sum(abs(g - target) for g in gaps[-1]) / 2, 3)
    result["low_factor_fraction"] = round(
        sum((r["factor2"] < 0.1) + (r["factor3"] < 0.1) for r in rows) / (2 * n), 3)
    result["repel_fraction"] = round(
        sum(bool(r["repel2"]) + bool(r["repel3"]) for r in rows) / (2 * n), 3)
    return result


def run_experiment(experiment_name, use_surprise):
    """
    Parse the command line, fly one convoy run, save CSV + JSON with metrics.

    Called from: main() here (E2) and surprise_minimization_experiment.py (E1).
    """
    parser = cli.make_parser(DEFAULT_CONFIG, f"Convoy experiment: {experiment_name}")
    args = parser.parse_args()
    config, seed = cli.load(args)
    observer = Observer(args.noise, seed)

    rows = []
    aborted = True   # stays True unless the flight finishes normally
    try:
        fly_safely(config, 3,
                   lambda drones, cfg, states: run_convoy(drones, cfg, states,
                                                          use_surprise, rows, observer))
        aborted = False
    finally:
        # Save whatever was recorded, even if the run was aborted.
        if rows:
            result = metrics(rows, config)
            # Files: data/raw/convoy_<baseline|surprise>_<mode>_<time>.csv/.json
            save_run(rows, config, "convoy", "surprise" if use_surprise else "baseline",
                     {"noise_std": args.noise, "seed": seed, "aborted": aborted,
                      "flown_s": rows[-1]["t"], "metrics": result})
            for key, value in result.items():
                print(f"  {key:22s} {value}")
            if aborted:
                print("NOTE: this run was ABORTED — exclude it when comparing.")
        else:
            print("No data recorded.")


def main():
    # use_surprise=False -> plain P-control (the baseline)
    run_experiment("convoy_baseline", use_surprise=False)


if __name__ == "__main__":
    main()
