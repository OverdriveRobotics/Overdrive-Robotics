#!/usr/bin/env bash
# Hardware-independent tests on the host JVM (real Ivy scheduler, proxy-mocked hardware, fake clock). SIMULATION ONLY.
#   scripts/run_tests.sh                 all suites
#   scripts/run_tests.sh turret flywheel run only those suites   (scripts/run_tests.sh --list shows them)
# JUnit is not in the offline Gradle cache, so a small runner (TeamCode/src/test/.../T.java, AllTests.java) is used.
source "$(dirname "$0")/_classpath.sh"
build_host_classes
cd "$ROOT"
if [ "${1:-}" = "--list" ]; then java -cp "$OUT:$CP" org.firstinspires.ftc.teamcode.AllTests --list; exit 0; fi
java -cp "$OUT:$CP" org.firstinspires.ftc.teamcode.AllTests "$@"
