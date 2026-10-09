#!/usr/bin/env bash
# Compiles the hardware-free control/command code + tests against the cached FTC/Pedro/Ivy jars and runs them
# on the host JVM (real Ivy scheduler, proxy-mocked hardware). JUnit is not in the offline Gradle cache.
# Run from the repo root:  scripts/run_tests.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
G="$HOME/.gradle/caches"
jar() { find "$G" -path "$1" -print -quit; }
CP=""
for pat in "*/RobotCore-12.0.0/jars/classes.jar" "*/Hardware-12.0.0/jars/classes.jar" "*/FtcCommon-12.0.0/jars/classes.jar" \
           "*/pedro-1.1.1/jars/classes.jar" "*/revhub-3.0.1/jars/classes.jar" "*/tuning-1.0.0/jars/classes.jar" "*/ftc-2.0.5/jars/classes.jar" "*/com.pedropathing/core/3.0.1/*/core-3.0.1.jar" "*/com.pedropathing.ivy/core/1.1.1/*/core-1.1.1.jar"; do
  j="$(jar "$pat")"; [ -n "$j" ] || { echo "missing jar: $pat" >&2; exit 2; }
  CP="$CP:$j"
done
OUT="${TMPDIR:-/tmp}/overdrive-tests"; rm -rf "$OUT"; mkdir -p "$OUT"
SRC="$ROOT/TeamCode/src/main/java/org/firstinspires/ftc/teamcode"
# Everything except OpModes/Constants (those need the Android/Pedro-config classpath).
FILES=$(find "$SRC/control" "$SRC/commands" "$SRC/hardware" -name '*.java'; find "$ROOT/TeamCode/src/test/java" -name '*.java')
javac -nowarn -d "$OUT" -cp "${CP#:}" $FILES "$ROOT/TeamCode/src/main/java/pedro/Constants.java" 2>&1 | grep -v "^Note:\|warning" || true
java -cp "$OUT:${CP#:}" org.firstinspires.ftc.teamcode.AllTests
