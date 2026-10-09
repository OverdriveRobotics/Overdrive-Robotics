#!/usr/bin/env bash
# Configuration manager for a directory pulled from the robot (see scripts/pull_config.sh).
#   scripts/config.sh DIR show|candidate|diff|history|params
#   scripts/config.sh DIR set KEY VALUE        edit the CANDIDATE (validated; safety limits refused)
#   scripts/config.sh DIR accept ["note"]      EXPLICIT: make the candidate active, then re-run the config + metrics tests
#   scripts/config.sh DIR reject | restore REV | good REV | restore-good | compare REPORT_A REPORT_B
source "$(dirname "$0")/_classpath.sh"
build_host_classes
cd "$ROOT"
set +e
java -cp "$OUT:$CP" org.firstinspires.ftc.teamcode.tuning.cli.ConfigTool "$@"
rc=$?
if [ $rc -eq 0 ] && [ "${2:-}" = "accept" ]; then
  echo "--- re-running the configuration tests after accept ---"
  java -cp "$OUT:$CP" org.firstinspires.ftc.teamcode.AllTests config metrics hardware workflow | tail -3
fi
exit $rc
