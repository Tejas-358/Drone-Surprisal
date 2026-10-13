"""
multi_hover_test.py — sanity check: can 1–3 REAL drones take off, hover and
land using MotionCommander (velocity / height control)?

Why this test exists next to takeoff_test.py:
  takeoff_test uses the high-level commander, which flies to ABSOLUTE
  positions and needs a Lighthouse position fix. MotionCommander instead
  holds a height above the floor and controls velocity, which works with
  a Flow deck alone. This test is for checking drones that have a Flow
  deck (or Lighthouse) before we rely on them.

Safety:
  - Before arming, every drone is checked for a Flow deck or Lighthouse
    deck. Without one the drone cannot measure its height or drift, so
    the test refuses to fly.
  - Whatever happens (error, Ctrl-C), every drone that took off is told to
    land, then motors are stopped, drones disarmed and links closed.

Run from the repo root (cfclient CLOSED — only one program can use the radio):
    python -m src.tests.multi_hover_test --drones 1
    python -m src.tests.multi_hover_test --drones 3
"""

import argparse
import time

from cflib.positioning.motion_commander import MotionCommander

from src.utils.connection import load_config, connect_all, disconnect_all
from src.utils.flight import stop_motors


HOVER_HEIGHT = 0.3     # m — kept low on purpose for first real flights
HOVER_SECONDS = 5.0    # how long to hover before landing


def check_positioning(drones):
    """
    Make sure every drone has a deck that can measure its position/height.

    Input:  drones — list from connect_all()
    Output: none. Raises an error (so nothing takes off) if any drone has
            neither a Flow deck nor a Lighthouse deck.

    Called from: main(), after connecting and BEFORE arming.
    """
    problems = []

    for i, scf in enumerate(drones):
        # The firmware sets these parameters to 1 if it detected the deck at boot.
        flow = int(scf.cf.param.get_value("deck.bcFlow2"))
        lighthouse = int(scf.cf.param.get_value("deck.bcLighthouse4"))
        print(f"CF{i + 1}: Flow deck={flow}  Lighthouse deck={lighthouse}")

        if not flow and not lighthouse:
            problems.append(f"CF{i + 1}")

    if problems:
        raise RuntimeError(
            f"No Flow or Lighthouse deck detected on {', '.join(problems)}. "
            "Not flying: the drone could not hold its height or position. "
            "Check the deck is mounted the right way round, then power-cycle."
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--drones", type=int, default=3, help="how many drones to fly")
    parser.add_argument("--mode", choices=["sim", "real"], help="override mode in config")
    args = parser.parse_args()

    config = load_config()
    if args.mode:
        config["mode"] = args.mode

    drones = connect_all(config, args.drones)
    commanders = []   # one MotionCommander per drone that we try to fly

    try:
        # CrazySim drones have no physical decks, so only check on hardware.
        if config["mode"] == "real":
            check_positioning(drones)

        # Recent firmware will not spin the motors until the drone is armed.
        print("\nArming...")
        for scf in drones:
            scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        # Take off one drone at a time. take_off() climbs to HOVER_HEIGHT and
        # then keeps the drone hovering in the background until land().
        for i, scf in enumerate(drones):
            mc = MotionCommander(scf, default_height=HOVER_HEIGHT)
            commanders.append(mc)   # added BEFORE take_off so it is landed even if take_off fails
            print(f"CF{i + 1} taking off to {HOVER_HEIGHT} m...")
            mc.take_off()

        print(f"\nAll {len(drones)} drones hovering for {HOVER_SECONDS:.0f} s...")
        time.sleep(HOVER_SECONDS)

    finally:
        # Runs on normal finish, error or Ctrl-C.
        print("\nLanding...")
        for i, mc in enumerate(commanders):
            try:
                mc.land()   # does nothing if this drone never took off
                print(f"CF{i + 1} landed.")
            except Exception as error:
                print(f"CF{i + 1} landing error: {error}")

        stop_motors(drones)       # motors off + disarm
        disconnect_all(drones)

    print("Done.")


if __name__ == "__main__":
    main()
