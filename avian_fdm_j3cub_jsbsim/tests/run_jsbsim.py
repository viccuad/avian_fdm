#!/usr/bin/env python3
"""Run JSBSim J3Cub with initial conditions matching avian_fdm's j3cub_minimal.

Outputs CSV to stdout: time_s,altitude_m,airspeed_ms,alpha_deg

Samples every 0.5 s from t = 0.5 to t = 60.0 (120 samples).

Uses the J3Cub FlightGear aircraft repo (revision 1.26, Datcom aero) rather
than the bare JSBSim-bundled aircraft. The repo lives at a sibling path named
J3Cub relative to the jsbsim root, e.g.:

    openskies/
        jsbsim/       <- JSBSIM_DATA_PATH
        J3Cub/        <- J3CUB_AIRCRAFT_PATH defaults to dirname(JSBSIM_DATA_PATH)

The J3Cub repo contains Engines/ and Systems/ subdirectories that JSBSim finds
automatically as aircraft-local resources when the aircraft path is set.

Only engine 0 (Continental A-65-8, 65 hp) is started. The J3Cub.xml in the
repo also defines a C90 (engine 1) and Lycoming O-320 (engine 2), which are
left off.

Environment:
    JSBSIM_DATA_PATH      JSBSim root (must contain engine/, systems/).
    J3CUB_AIRCRAFT_PATH   Parent directory of the J3Cub folder.
                          Defaults to dirname(JSBSIM_DATA_PATH).
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
    # Resolve JSBSim root (for shared engine/ and systems/ data).
    data_path = os.environ.get("JSBSIM_DATA_PATH", "")
    if not data_path:
        print("ERROR: JSBSIM_DATA_PATH not set.", file=sys.stderr)
        sys.exit(1)
    data_path = os.path.abspath(data_path)

    # Resolve aircraft path: parent directory that contains the J3Cub folder.
    # Default: sibling of the jsbsim root (e.g. openskies/J3Cub lives next to openskies/jsbsim).
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

    # JSBSim's C++ code prints a version banner to the C-level stdout during
    # construction.  Temporarily redirect the POSIX file descriptor so it
    # doesn't corrupt our CSV output.
    saved_fd = os.dup(1)
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull_fd, 1)
    os.close(devnull_fd)

    fdm = jsbsim.FGFDMExec(data_path, None)
    fdm.set_debug_level(0)
    fdm.set_aircraft_path(aircraft_path)

    # Restore stdout.
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

    # Start only engine 0 (Continental A-65-8, 65 hp).
    # The J3Cub.xml also defines engine 1 (C90) and engine 2 (Lycoming O-320),
    # which are left off. Using index [0] instead of set-running = -1 avoids
    # spurious thrust from the unused variants.
    fdm["propulsion/engine[0]/set-running"] = 1
    fdm["fcs/throttle-cmd-norm[0]"] = 0.5
    fdm["fcs/mixture-cmd-norm[0]"] = 1.0

    # ── Simulation loop ──────────────────────────────────────────────────
    print("time_s,altitude_m,airspeed_ms,alpha_deg")

    dt = fdm.get_delta_t()
    next_sample = SAMPLE_INTERVAL_S

    while True:
        t = fdm["simulation/sim-time-sec"]
        if t > SIM_DURATION_S + dt:
            break

        # Throttle ramp: 50% to 75% over the first 12.5 s, then hold.
        throttle = min(0.5 + t / 50.0, 0.75)
        fdm["fcs/throttle-cmd-norm[0]"] = throttle

        if t >= next_sample - dt / 2.0:
            alt_m = fdm["position/h-agl-ft"] / FT_PER_M
            tas_ms = fdm["velocities/vt-fps"] / FT_PER_M
            alpha_deg = fdm["aero/alpha-rad"] * RAD_TO_DEG

            print(f"{t:.4f},{alt_m:.6f},{tas_ms:.6f},{alpha_deg:.6f}")
            next_sample += SAMPLE_INTERVAL_S

        fdm.run()


if __name__ == "__main__":
    main()
