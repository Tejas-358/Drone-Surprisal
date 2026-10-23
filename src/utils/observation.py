"""
observation.py — what the CONTROLLER sees: the drone's state, optionally
with extra random noise added.

Why: the simulator is perfectly repeatable, so 5 runs of the same setting
give 5 identical runs and averages mean nothing. Real Lighthouse positions
also wobble (about +-2-4 cm). Adding Gaussian noise to what the controller
sees makes sim runs differ like real ones; the random seed makes each noisy
run repeatable.

  noise_std = 0   -> exact state (default; use this on real drones)
  noise_std > 0   -> x, y, z (m) and vx, vy, vz (m/s) each get Gaussian
                     noise with this standard deviation

The SAFETY checks always use the true state — only control decisions see
the noise.

Used by: the control loops of every experiment (--noise / --seed options).
"""

import random


class Observer:
    """Gives noisy copies of drone states. One per run."""

    def __init__(self, noise_std, seed):
        """
        Input:  noise_std — standard deviation of the added noise (0 = none)
                seed      — random seed, so a noisy run can be repeated exactly
        """
        self.noise_std = noise_std
        self.rng = random.Random(seed)

    def see(self, state):
        """
        Return a copy of one drone's state with noise added.

        Input:  state — a telemetry dictionary from sensors.py
        Output: new dictionary with the same keys (the original is untouched).
        """
        seen = dict(state)
        if self.noise_std > 0:
            for key in ("x", "y", "z", "vx", "vy", "vz"):
                seen[key] = state[key] + self.rng.gauss(0.0, self.noise_std)
        return seen
