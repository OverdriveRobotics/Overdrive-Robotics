# Calibration guide - order of operations for the assembled robot

Read [`calibration/README.md`](../../../../../../../../../calibration/README.md) first (labels, workflow, safety). Everything below uses the guided
"CAL ..." TeleOps and is **HARDWARE** work unless stated. Do each stage fully (including accept) before the next. After every
accept: `scripts/config.sh DIR accept` on the PC re-runs the configuration tests; commit nothing to the robot until they pass.

## Before the robot exists / before you touch it (HARDWARE-INDEPENDENT)
1. `scripts/run_tests.sh` - all suites pass.
2. `./gradlew :TeamCode:assembleDebug` - builds.
3. Optional rehearsal of the tools: `scripts/sim_calibrate.sh /tmp/rehearsal flywheel-step` (SIMULATION).

## Stage 0 - bring-up (robot on blocks, nothing powered by the framework)
* `CAL Config Manager`: confirm *active rev 0* and that no `CONFIG PROBLEMS` are shown.
* `CAL Localization` #1-#3: pose, heading, distance sanity; #1 decides `turret.ROBOT_VELOCITY_IS_FIELD_FRAME` (software says field-frame; this confirms the raw Pinpoint output).
* `CAL Localization` #5: measure target points A and B -> **Y** -> accept.

## Stage 1 - mechanisms with no ball
* `CAL Stopper` #1 endpoints (slow jog), #2 timing. Accept.
* `CAL Intake` #1 direction + transfers (no balls: judge direction only; then with balls). Accept power/threshold changes.

## Stage 2 - flywheel
* `CAL Flywheel` #1 scaling -> #2 sysid -> **Y** -> add `flywheel.MODEL_IDENTIFIED` to the candidate -> #3 step, #4 disturbance under the candidate (**A**) vs the active run (**X**).
* Accept only if the candidate is better on the robot. Otherwise reject: the SDK PIDF path stays.

## Stage 3 - turret (motor stays disabled until this stage ends)
* `CAL Turret` #1 direction -> #2 ticks/rev -> #3 limits -> enter motor datasheet values and inertia/friction (#5 or CAD) -> **Y** -> `turret.HARDWARE_CONFIGURED=1` in the candidate (the validator names anything still missing) -> accept.
* #6 step, #7 aim accuracy (record `observed_aim_offset_deg`). Tune the LQR weights one at a time; rerun and compare.

## Stage 4 - readiness gates
* `CAL Shooting Window`: define the zone and speed limit; run *monitor only*, then *shoot 1*, then *shoot 3*. Enable `REQUIRE_LOCALIZATION`, `SHOOT_ZONE_ENABLED`, then `REQUIRE_TURRET_ALIGNED`, one at a time.

## Stage 5 - moving shot (the robot drives)
* `CAL Moving Shot` stages 1 -> 6 in order, at slow path speeds first. Stage N must be clean before N+1.

## When something looks wrong
* Compare with the baseline (**X** reruns the active settings under identical conditions).
* Restore: `CAL Config Manager` -> *Restore latest known-good*, or `scripts/config.sh DIR restore-good`.
* The robot ignores an invalid configuration file as a whole and says so in telemetry (`CONFIG PROBLEMS`).

## What stays your decision
Motor directions and wiring (source), mechanical limits' *safety margins*, voltage/power caps (source), autonomous paths, and whether a candidate is better *enough* to accept. The framework refuses unsafe values; it cannot judge fitness for a match.
