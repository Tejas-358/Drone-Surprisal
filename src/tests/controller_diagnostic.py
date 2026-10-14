
# Save as src/tests/controller_diagnostic.py

from src.utils.connection import load_config, connect_all, disconnect_all

def main():
    config = load_config()
    config["mode"] = "real"

    drones = connect_all(config, 1)

    try:
        cf = drones[0].cf

        for name in (
            "stabilizer.controller",
            "stabilizer.estimator",
        ):
            try:
                value = cf.param.get_value(name)
                print(f"{name} = {value}")
            except Exception as exc:
                print(f"{name}: unable to read ({exc})")

        print("\nDiagnostic only: no takeoff or movement commands sent.")

    finally:
        disconnect_all(drones)

if __name__ == "__main__":
    main()
