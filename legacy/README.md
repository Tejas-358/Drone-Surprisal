# legacy/

The original flat-layout scripts, kept **unchanged** as a known-good reference
while the code is migrated into `src/`. Do not edit them.

| Old script | New home |
|---|---|
| `test_connection.py` | `src/tests/test_connection.py` |
| `takeoff_test.py` | `src/tests/takeoff_test.py` |
| `three_takeoff_test.py` | `src/tests/three_takeoff_test.py` |
| `repeat_test.py` | `src/tests/hover_telemetry_test.py` |
| `surprise_minimization_experiment.py` | E1 — not migrated yet |
| `wall_follow_experiment.py` | E2 convoy baseline — not migrated yet |
| `distance_controller.py` | E3 formation distance — not migrated yet |
| `formation_test.py` | not migrated yet |

Their old CSV outputs are in `data/raw/legacy/`.

They can still be run from the repo root, e.g. `python legacy/takeoff_test.py`
(note: they write their CSVs to the current folder). Each file is deleted once
its new version has been confirmed in the simulator.
