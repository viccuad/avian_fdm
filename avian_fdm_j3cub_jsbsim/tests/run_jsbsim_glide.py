#!/usr/bin/env python3
"""Run JSBSim J3Cub in unpowered glide, matching avian_fdm's no-engine scenario.

Outputs CSV to stdout: time_s,altitude_m,airspeed_ms,alpha_deg

Same initial conditions as run_jsbsim.py, but no engine is started and
throttle stays at 0. Used to generate tests/fixtures/jsbsim_j3cub_glide_60s.csv.

See run_jsbsim.py for path setup (JSBSIM_DATA_PATH, J3CUB_AIRCRAFT_PATH).
"""

import math
import os
import sys

try:
    import jsbsim
except ImportError:
    print("ERROR: jsbsim package not installed. Run: pip install jsbsim",
          file=sys.stderr)
    sys.exit(1)

# ── Constants matching avian_fdm j3cub ───────────────────────────────────────
INITIAL_ALT_M = 300.0
INITIAL_TAS_MS = 27.0
SIM_DURATION_S = 60.0
SAMPLE_INTERVAL_S = 0.5
FT_PER_M = 3.28084
RAD_TO_DEG = 180.0 / math.pi


def main():
    data_path = os.environ.get("JSBSIM_DATA_PATH", "")
    if not data_path:
        print("ERROR: JSBSIM_DATA_PATH not set.", file=sys.stderr)
        sys.exit(1)
    data_path = os.path.abspath(data_path)

    aircraft_path = os.environ.get(
        "J3CUB_AIRCRAFT_PATH", os.path.dirname(data_path)
    )
    aircraft_path = os.path.abspath(aircraft_path)

    j3cub_dir = os.path.join(aircraft_path, "J3Cub")
    if not os.path.isdir(j3cub_dir):
        print(
            f"ERROR: J3Cub aircraft directory not found at {j3cub_dir}.\n"
            "Set J3CUB_AIRCRAFT_PATH to the directory containing the J3Cub folder.",
            file=sys.stderr,
        )
        sys.exit(1)

    saved_fd = os.dup(1)
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull_fd, 1)
    os.close(devnull_fd)

    fdm = jsbsim.FGFDMExec(data_path, None)
    fdm.set_debug_level(0)
    fdm.set_aircraft_path(aircraft_path)

    os.dup2(saved_fd, 1)
    os.close(saved_fd)

    if not fdm.load_model("J3Cub"):
        print(
            f"ERROR: Failed to load J3Cub model.\n"
            f"  JSBSIM_DATA_PATH    = {data_path}\n"
            f"  J3CUB_AIRCRAFT_PATH = {aircraft_path}",
            file=sys.stderr,
        )
        sys.exit(1)

    # ── Initial conditions ───────────────────────────────────────────────
    fdm["ic/h-agl-ft"] = INITIAL_ALT_M * FT_PER_M
    fdm["ic/vt-fps"] = INITIAL_TAS_MS * FT_PER_M
    fdm["ic/theta-rad"] = 0.0
    fdm["ic/phi-rad"] = 0.0
    fdm["ic/psi-true-rad"] = 0.0
    fdm["ic/terrain-elevation-ft"] = 0.0

    fdm.run_ic()

    # Engine is left off - no set-running call, throttle and mixture stay at 0.
    fdm["fcs/throttle-cmd-norm[0]"] = 0.0
    fdm["fcs/mixture-cmd-norm[0]"] = 0.0

    # ── Simulation loop ──────────────────────────────────────────────────
    print("time_s,altitude_m,airspeed_ms,alpha_deg")

    dt = fdm.get_delta_t()
    next_sample = SAMPLE_INTERVAL_S

    while True:
        t = fdm["simulation/sim-time-sec"]
        if t > SIM_DURATION_S + dt:
            break

        if t >= next_sample - dt / 2.0:
            alt_m = fdm["position/h-agl-ft"] / FT_PER_M
            tas_ms = fdm["velocities/vt-fps"] / FT_PER_M
            alpha_deg = fdm["aero/alpha-rad"] * RAD_TO_DEG

            print(f"{t:.4f},{alt_m:.6f},{tas_ms:.6f},{alpha_deg:.6f}")
            next_sample += SAMPLE_INTERVAL_S

        fdm.run()


if __name__ == "__main__":
    main()
