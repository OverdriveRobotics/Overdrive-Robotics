#!/usr/bin/env bash
# Offline (SIMULATION) weight search; writes a CANDIDATE only.   scripts/offline_tune.sh DIR flywheel 1800   |   scripts/offline_tune.sh DIR turret
source "$(dirname "$0")/_classpath.sh"
build_host_classes
cd "$ROOT"
java -cp "$OUT:$CP" org.firstinspires.ftc.teamcode.tuning.cli.OfflineTuneCli "$@"
