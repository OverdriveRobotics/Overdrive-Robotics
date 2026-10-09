# sourced by the other scripts: builds CP (cached FTC/Pedro/Ivy jars) and OUT (compiled classes). Not executable on its own.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
G="$HOME/.gradle/caches"
jar() { find "$G" -path "$1" -print -quit; }
CP=""
for pat in "*/RobotCore-12.0.0/jars/classes.jar" "*/Hardware-12.0.0/jars/classes.jar" "*/FtcCommon-12.0.0/jars/classes.jar" \
           "*/pedro-1.1.1/jars/classes.jar" "*/revhub-3.0.1/jars/classes.jar" "*/tuning-1.0.0/jars/classes.jar" "*/ftc-2.0.5/jars/classes.jar" \
           "*/com.pedropathing/core/3.0.1/*/core-3.0.1.jar" "*/com.pedropathing.ivy/core/1.1.1/*/core-1.1.1.jar"; do
  j="$(jar "$pat")"; [ -n "$j" ] || { echo "missing jar: $pat (run a Gradle build once so the dependencies are cached)" >&2; exit 2; }
  CP="$CP:$j"
done
CP="${CP#:}"
OUT="${TMPDIR:-/tmp}/overdrive-tests"
SRC="$ROOT/TeamCode/src/main/java/org/firstinspires/ftc/teamcode"
build_host_classes() {
  rm -rf "$OUT"; mkdir -p "$OUT"
  # OpModes and pedro/Constants need the Android runtime; the host build covers control, commands, hardware, tuning, calibration (+ tests).
  local files
  files=$(find "$SRC/control" "$SRC/commands" "$SRC/hardware" "$SRC/tuning" "$SRC/calibration" -not -path "*/opmodes/*" -name '*.java'; find "$ROOT/TeamCode/src/test/java" -name '*.java')
  javac -nowarn -d "$OUT" -cp "$CP" $files "$ROOT/TeamCode/src/main/java/pedro/Constants.java" 2>&1 | grep -v "^Note:\|warning" || true
}
