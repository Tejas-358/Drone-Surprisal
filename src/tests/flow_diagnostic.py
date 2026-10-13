import time
import cflib.crtp

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper


URI = uri_helper.uri_from_env(
    default="radio://0/80/2M/E7E7E7E7E7"
)


def position_callback(timestamp, data, logconf):
    x = data["stateEstimate.x"]
    y = data["stateEstimate.y"]
    z = data["stateEstimate.z"]

    print(
        f"X = {x:7.3f} m | "
        f"Y = {y:7.3f} m | "
        f"Z = {z:7.3f} m"
    )


def main():

    cflib.crtp.init_drivers()

    print(f"Connecting to {URI}...")

    cf = Crazyflie(rw_cache="./cache")

    with SyncCrazyflie(URI, cf=cf) as scf:

        print("Connected to Crazyflie")
        print("Reading Lighthouse position...")
        print("Press Ctrl+C to stop.\n")

        logconf = LogConfig(
            name="LighthousePosition",
            period_in_ms=100
        )

        logconf.add_variable(
            "stateEstimate.x",
            "float"
        )

        logconf.add_variable(
            "stateEstimate.y",
            "float"
        )

        logconf.add_variable(
            "stateEstimate.z",
            "float"
        )

        logconf.data_received_cb.add_callback(
            position_callback
        )

        scf.cf.log.add_config(logconf)

        logconf.start()

        try:
            while True:
                time.sleep(1)

        except KeyboardInterrupt:
            print("\nStopping...")

        finally:
            logconf.stop()


if __name__ == "__main__":
    main()