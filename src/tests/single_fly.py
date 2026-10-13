import time
import cflib.crtp

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander



# CF1 URI
URI = "radio://0/80/2M/E7E7E7E7E6"

# Initialize radio drivers
cflib.crtp.init_drivers()

print("Connecting to CF1...")

try:
    with SyncCrazyflie(
        URI,
        cf=Crazyflie(rw_cache="./cache")
    ) as scf:

        print("Connected to CF1!")

        # Arm the drone
        scf.cf.platform.send_arming_request(True)
        time.sleep(1)

        # Take off, hover, and land
        with MotionCommander(
            scf,
            default_height=0.3
        ) as mc:

            print("Taking off...")
            time.sleep(2)

            print("Hovering for 5 seconds...")
            time.sleep(5)

            print("Landing...")

        print("Landed successfully!")

except Exception as e:
    print(f"Error: {e}")

print("Flight program finished.")

