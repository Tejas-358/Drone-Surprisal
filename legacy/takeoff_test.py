import time
import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = "udp://127.0.0.1:19850"


def main():

    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache="./cache")

    with SyncCrazyflie(URI, cf) as scf:

        print("Connected to CF1")

        commander = scf.cf.high_level_commander

        print("Taking off...")
        commander.takeoff(0.5, 2.0)

        time.sleep(3)

        print("Hovering...")
        time.sleep(3)

        print("Landing...")
        commander.land(0.0, 2.0)

        time.sleep(3)

        print("Done.")


if __name__ == "__main__":
    main()