#!/usr/bin/env bash
# Run a calibration procedure against the SIMULATOR through the real calibration workflow (reports, CSV logs, candidate).
#   scripts/sim_calibrate.sh DIR flywheel-step
#   scripts/sim_calibrate.sh DIR flywheel-sysid --adopt          # writes DIR/candidate.properties (never active)
#   scripts/sim_calibrate.sh DIR flywheel-step --set robot.FLYWHEEL_STABLE_MS=400   # rerun under a candidate and compare with the first run
# SIMULATION ONLY. See calibration/README.md.
source "$(dirname "$0")/_classpath.sh"
build_host_classes
cd "$ROOT"
java -cp "$OUT:$CP" org.firstinspires.ftc.teamcode.commands.SimCalibrate "$@"
