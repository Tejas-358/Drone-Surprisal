"""
cli.py — the command-line options shared by every experiment.

    --mode sim|real        which drones to use (overrides config/default.yaml)
    --config FILE          the experiment's YAML file
    --noise STD            Gaussian noise (m, m/s) on what the controller sees
                           (0 = none; useful in sim to mimic real Lighthouse jitter)
    --seed N               random seed for that noise (random if not given)
    --set key=value        change any config value for this run, e.g.
                           --set noise_scale=0.1   (can be repeated; used for sweeps)

Used by: main() of every experiment in src/experiments/.
"""

import argparse
import random

import yaml

from src.utils.connection import load_config


def make_parser(default_config, description):
    """
    Build an argument parser with the shared options. An experiment can add
    its own options (e.g. --controller) before parsing.
    """
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--mode", choices=["sim", "real"],
                        help="override `mode` in config/default.yaml")
    parser.add_argument("--config", default=default_config, help="experiment YAML file")
    parser.add_argument("--noise", type=float, default=0.0,
                        help="std of noise added to what the controller sees (default 0)")
    parser.add_argument("--seed", type=int, help="random seed for the noise")
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                        help="override a config value, e.g. --set noise_scale=0.1")
    return parser


def load(args):
    """
    Turn parsed arguments into (config, seed).

    The config is default.yaml + the experiment YAML + any --set overrides.
    The seed is the one given, or a random one (it is saved with the run,
    so any noisy run can be repeated exactly).
    """
    config = load_config(args.config)
    if args.mode:
        config["mode"] = args.mode

    for item in args.set:
        key, value = item.split("=", 1)
        if key not in config:
            raise SystemExit(f"--set: unknown config key '{key}'")
        config[key] = yaml.safe_load(value)   # "0.1" -> 0.1, "[1, 2]" -> [1, 2]

    seed = args.seed if args.seed is not None else random.randrange(1_000_000)
    return config, seed
