#!/usr/bin/env bash
# Analyse a CSV log pulled from the robot.   scripts/analyze_log.sh sysid LOG.csv   |   step LOG.csv TARGET BAND   |   show LOG.csv
source "$(dirname "$0")/_classpath.sh"
build_host_classes
cd "$ROOT"
java -cp "$OUT:$CP" org.firstinspires.ftc.teamcode.tuning.cli.LogAnalyzer "$@"
