import time
import cflib

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig


URI = "udp://127.0.0.1:19850"


def main():

    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache="./cache")

    with SyncCrazyflie(URI, cf) as scf:

        print("Connected to CF1")

        positions = {}

        def callback(timestamp, data, logconf):
            positions["x"] = data["stateEstimate.x"]
            positions["y"] = data["stateEstimate.y"]
            positions["z"] = data["stateEstimate.z"]

        log_config = LogConfig(
            name="Position",
            period_in_ms=100
        )

        log_config.add_variable("stateEstimate.x", "float")
        log_config.add_variable("stateEstimate.y", "float")
        log_config.add_variable("stateEstimate.z", "float")

        log_config.data_received_cb.add_callback(callback)

        scf.cf.log.add_config(log_config)
        log_config.start()

        time.sleep(1)

        commander = scf.cf.high_level_commander

        print("TAKEOFF")

        commander.takeoff(0.5, 2.0)

        for i in range(30):

            time.sleep(0.2)

            if positions:
                print(
                    f"t={i * 0.2:.1f}s  "
                    f"x={positions['x']:.3f}  "
                    f"y={positions['y']:.3f}  "
                    f"z={positions['z']:.3f}"
                )

        print("LAND")

        commander.land(0.0, 2.0)

        time.sleep(3)

        log_config.stop()

        print("Finished")


if __name__ == "__main__":
    main()