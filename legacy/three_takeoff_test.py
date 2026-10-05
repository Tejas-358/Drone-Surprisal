import time
import cflib

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


URIS = [
    "udp://127.0.0.1:19850",
    "udp://127.0.0.1:19851",
    "udp://127.0.0.1:19852",
]


def main():

    cflib.crtp.init_drivers()

    drones = []

    # Connect
    for i, uri in enumerate(URIS):

        print(f"Connecting to CF{i + 1}...")

        cf = Crazyflie(rw_cache="./cache")
        scf = SyncCrazyflie(uri, cf)

        scf.open_link()

        drones.append(scf)

        print(f"CF{i + 1} connected")

    try:

        # Take off all three
        print("\nTaking off all three drones...")

        for scf in drones:
            scf.cf.high_level_commander.takeoff(
                0.5,
                2.0
            )

        time.sleep(4)

        print("All three should be hovering.")

        time.sleep(5)

        # Land all three
        print("Landing...")

        for scf in drones:
            scf.cf.high_level_commander.land(
                0.0,
                2.0
            )

        time.sleep(3)

        print("Done.")

    finally:

        for scf in drones:
            scf.close_link()


if __name__ == "__main__":
    main()