#!/usr/bin/env bash
# Copy the robot's calibration directory to this PC (needs adb and the Control Hub / phone connected). Not run by any test.
#   scripts/pull_config.sh LOCAL_DIR          pull   /sdcard/FIRST/overdrive  ->  LOCAL_DIR
#   scripts/pull_config.sh LOCAL_DIR push     push   LOCAL_DIR/active.properties (+ candidate, history) -> robot. Review 'config.sh LOCAL_DIR show' first.
set -euo pipefail
command -v adb >/dev/null || { echo "adb not found (install Android platform-tools; for a Control Hub: adb connect 192.168.43.1:5555)"; exit 2; }
DIR="${1:?usage: pull_config.sh LOCAL_DIR [push]}"
REMOTE=/sdcard/FIRST/overdrive
if [ "${2:-}" = "push" ]; then
  echo "pushing $DIR -> $REMOTE (only configuration files; run logs stay on the PC)"
  adb shell mkdir -p $REMOTE
  for f in active.properties candidate.properties known-good.txt; do [ -f "$DIR/$f" ] && adb push "$DIR/$f" $REMOTE/ || true; done
  [ -d "$DIR/history" ] && adb push "$DIR/history" $REMOTE/ || true
else
  mkdir -p "$DIR"; adb pull $REMOTE/. "$DIR"
fi
