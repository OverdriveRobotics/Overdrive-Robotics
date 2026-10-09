# Test and calibration framework - verification report

Date: 2026-10-09. Everything below was executed in this repository unless it says otherwise. **No physical robot was available:
nothing in this report validates hardware behaviour.**

## 1. Results that were actually executed

| Command | Result |
|---|---|
| `scripts/run_tests.sh` (all hardware-independent suites, real Ivy 1.1.1 + Pedro 3.0.1 classes, mocked hardware, fake clock) | **190 passed, 0 failed** |
| `./gradlew :TeamCode:compileDebugJavaWithJavac` | success |
| `./gradlew :TeamCode:compileDebugUnitTestJavaWithJavac` (Android Studio compatibility of the test sources) | success |
| `./gradlew :TeamCode:assembleDebug` | success (the OpModes, including the 8 calibration OpModes, compile and dex) |
| `./gradlew :TeamCode:hostTests -Psuites="ivy hardware"` (Gradle wrapper around the runner) | 20 passed, 0 failed |
| `scripts/sim_calibrate.sh`, `config.sh`, `offline_tune.sh` end-to-end demonstration (section 5) | ran as described |

Per suite (`scripts/run_tests.sh --list` names them):

| Suite | Result |
|---|---|
| `config` | 16 passed, 0 failed |
| `metrics` | 9 passed, 0 failed |
| `control` | 20 passed, 0 failed |
| `readiness` | 5 passed, 0 failed |
| `turret-math` | 9 passed, 0 failed |
| `flywheel-observer` | 8 passed, 0 failed |
| `flywheel` | 11 passed, 0 failed |
| `shooting` | 14 passed, 0 failed |
| `validation` | 2 passed, 0 failed |
| `intake` | 5 passed, 0 failed |
| `turret` | 5 passed, 0 failed |
| `movingshot` | 2 passed, 0 failed |
| `cal-flywheel` | 7 passed, 0 failed |
| `cal-turret` | 9 passed, 0 failed |
| `cal-intake-stopper` | 5 passed, 0 failed |
| `cal-localization` | 8 passed, 0 failed |
| `cal-window-movingshot` | 6 passed, 0 failed |
| `workflow` | 9 passed, 0 failed |
| `hardware` | 7 passed, 0 failed |
| `ivy` | 13 passed, 0 failed |
| `state` | 7 passed, 0 failed |
| `opmode-exit` | 1 passed, 0 failed |
| `auto-routines` | 4 passed, 0 failed |
| `shoot-cancel` | 6 passed, 0 failed |
| `lifecycle` | 2 passed, 0 failed |

JUnit is not in the offline Gradle cache, so the suites use a small runner instead of JUnit; the Gradle task `hostTests` makes them runnable from Android Studio.

### What is NOT executed / validated
* **The OpModes never ran** (they cannot be instantiated off-robot). They compile and dex. `opmode-exit` is a **static source scan** of their exit paths (`Scheduler.reset()` then `safeShutdown(...)`), not an execution.
* All 8 `CAL ...` OpModes, their gamepad menu, telemetry and prompts: **compile-only**. The procedure logic they call is tested in simulation through an identical `CalContext` interface; the I/O wrapper is not.
* `RobotHardware.init` with a real `HardwareMap`, real motors/encoders/servo, real Pedro/Pinpoint accuracy, real shot dynamics, real servo timing, real battery behaviour.
* The simulated "SDK velocity PIDF" is an arbitrary first-order model, so any simulated LQR-vs-SDK comparison says **nothing** about the robot. No evidence exists that LQR beats the SDK PIDF (the controller tests show it equals feed-forward + P with the same gain).

## 2. Files

**New production code:** `tuning/` (Param, ParamRegistry, ConfigValidator, ConfigStore, Tuning, RunLog, RunReport, Metrics, SysId, WindowTracker, VelocityFrame, OfflineTuner, `cli/{ConfigTool,LogAnalyzer,OfflineTuneCli}`),
`calibration/` (CalContext, CalConfig, CalSettings, Procedure, ProcedureResult, Preflight, CalSession, ParamEditor, Flywheel/Turret/Intake/Stopper/Localization/ShootWindow/MovingShot procedures, TurretCalLogic, `opmodes/` x9), `commands/AutoRoutines`.
**Modified production code:** `RobotHardware` (loads the active configuration at init), `Storage` (disturbance hook, raw target setter), `FlywheelRegulator` (calibration disturbance hook), `RobotConfig`/`TurretConfig`/`FlywheelConfig` (tunable fields are no longer `final`), `RobotTelemetry` (config revision), `Lqr` (bug fix, below), `MovingShotAutoExample` (uses `AutoRoutines`), `TeamCode/build.gradle` (`hostTests` task).
**Tests (31 files):** `commands/` (simulator `SimRobot`, `SimCalContext`, `SimCalibrate`, `SimSupport` + 20 suites), `control/` (4 suites), `tuning/` (2 suites), `AllTests`, `T`.
**Scripts:** `run_tests.sh`, `sim_calibrate.sh`, `config.sh`, `offline_tune.sh`, `analyze_log.sh`, `pull_config.sh`, `gen_param_docs.sh`, `_classpath.sh`.
**Docs:** `calibration/README.md` + 9 subsystem READMEs, `calibration/templates/*`, `docs/{CALIBRATION_GUIDE,IVY_GUIDE,PARAMETERS,SUBSYSTEM_INVENTORY,TEST_REPORT}.md`, `tests/README.md`; `MOVING_SHOT_SYSTEM.md` updated.
`CommandTests.java` was split into per-subsystem suites (no test lost: the original 54 tests are all still present).

## 3. Bugs and problems found by this work (and fixed)

Production code (1 defect, 1 corrected assumption, 1 bug in a new procedure, 1 design addition):
1. **Kalman Riccati iteration never converged for large `MODEL_STD`** (absolute tolerance): `FlywheelLqrController` threw at regulator start for legitimately tunable values (hundreds). Now relative tolerance.
2. *Corrected assumption* - **Pedro's velocity frame** was unverified: now verified from the installed bytecode (`PinpointLocalizer` -> `MotionState.ofVelocity` => field-frame) and cross-checked numerically against Pedro's `Twist.toVelocity`; the production flag kept, comment corrected.
3. **Moving-shot/window procedures counted a stale `ballsShot`** from the previous command (stage 6 reported 3 shots instead of 2): now a delta of the cumulative counter.
4. *Design addition* - **calibration disturbance injection** needed a controlled way to disturb the single-writer regulator: added a guarded, normally-zero hook instead of a second motor writer.

Tooling (found by testing the new tools):
5. System identification: forward-difference regression had a ~1.4 % bias; replaced by the exact zero-order-hold regression. A log-alignment convention (voltage applied over the interval *ending* at the sample) is now documented and tested.
6. System identification silently returned garbage (negative gain) when only **one drive voltage** was used (static friction and gain are not separable). Detected now (kS reported as 0, `kSIdentified=false`) and the turret procedure uses several pulse levels.
7. Identification "R^2" scored noisy one-step increments (a good fit read 0.46): now a **replay** of the fitted model over the whole recorded trajectory.
8. `WindowTracker` attributed the blocking reason of the *new* sample to the interval that just ended ("ready" appeared as a blocking reason).
9. Report comparison judged a signed steady-state error of -4.6e-6 vs -4.4e-6 as WORSE: now compares magnitudes with a noise floor.
10. Baselines were lost between separate runs (no comparison after restarting a tool): now loaded from the earliest earlier report on disk.
11. The first unit-test sources would not have compiled under Android Studio (`sun.misc`); found by running `compileDebugUnitTestJavaWithJavac`, now reflective.
12. `LinearOpMode.loop()` is final: the context method is `tick()`.
13. Simulator fidelity: sensors were one sample stale after `loop()`, which made the turret identification 35 % wrong; fixed in the simulator (the real robot reads fresh state).

## 4. Calibration tools available, parameters, and what still needs the robot

| Subsystem | Tool | Adjustable (examples; full list in `docs/PARAMETERS.md`) | Still required from the physical robot |
|---|---|---|---|
| Turret | `CAL Turret` #1-#7 | ticks/rev, external ratio, encoder sign, start angle, motor datasheet values, inertia/friction, LQR weights, slew, filter, integral, alignment tolerances, targets; limits measured-only | **everything**: motor model, gearing, encoder resolution, limits, inertia/friction, real aim offset |
| Flywheel | `CAL Flywheel` #1-#4 | tolerance, stability, velocities, LQR weight, Kalman std-devs, kS/kV/kA, ticks/rev, `MODEL_IDENTIFIED` | motor/encoder facts, tachometer reading, identified model, step/disturbance results, SDK-PIDF baseline, real ball drop |
| Intake / transfer | `CAL Intake` | intake/feed power, jam enable/threshold/time | direction, real currents, transfer reliability |
| Stopper | `CAL Stopper` | open/closed positions, settle times | real positions and timings (operator-observed) |
| Localization | `CAL Localization` #1-#5 | velocity-frame flag, target points | raw Pinpoint velocity frame, odometry scale/heading error, target coordinates |
| Shooting window | `CAL Shooting Window` | zone, speed limit, alignment tolerances, timeouts, drop threshold | zone geometry, achievable window length |
| Moving shot | `CAL Moving Shot` stages 1-6 | test path (unset by default), balls, window, time limit | all stages |
| Ivy | automated tests + `AutoRoutines` + `IVY_GUIDE.md` | - (nothing physical) | - |
| Configuration | `CAL Config Manager`, `scripts/config.sh` | candidate / accept / reject / restore / known-good | - |

Also still open from before: target points A/B, zone, per-target velocities, SDK PIDF values in `RobotHardware` (all `1,1,1,1`), Pedro Foresight tuning.

## 5. Calibration runs (all **SIMULATION**; they exercise the tools, not the robot)

End-to-end CLI demonstration (`scripts/sim_calibrate.sh`, `scripts/config.sh`), simulated flywheel:

```
1  flywheel-step (SDK-PIDF path):     settling 0.91 s, readiness 1.08 s, overshoot 0.75 %
2  flywheel-sysid --adopt:            kV 0.004379 (plant 0.004286 in the config default, simulator truth 0.00438),
                                      kA 0.002676, replay r2 0.99995  -> written to candidate.properties (NOT active)
3  config.sh diff:                    KA / KS / KV / USE_SYSID_MODEL changes listed
4  rerun with robot.FLYWHEEL_STABLE_MS=400 under the candidate:
                                      readiness_delay 1.08 -> 1.33 s  WORSE;  parameter changes listed; compared with the first run FROM DISK
5  set flywheel.MODEL_IDENTIFIED 1 (accepted by the validator only because KV/KA/KS are now explicit), rerun:
                                      LQR path: settling 0.91 -> 0.62 s, steady-state error 0.39 -> 0.06 (simulated; not evidence about the robot),
                                      saturation_fraction 0 -> 0.105 WORSE, overshoot 0.75 -> 0.86 % WORSE
6  config.sh accept "..."            -> revision 1, config tests re-run automatically (40 passed), history shows rev 1
7  unsafe edits:                      `set turret.MAX_CONTROL_VOLTS 2` refused (safety), `set turret.HARDWARE_CONFIGURED 1` refused with 10 missing-calibration reasons
8  offline_tune flywheel 1800:        search over 36 weight pairs; best == current default (score 11.93): the default is already on the grid optimum for the simulated plant; candidate written, no difference
```

Other simulated procedures (headline numbers; simulator ground truth in brackets where known):

| Procedure | Headline result (simulation) |
|---|---|
| flywheel-scaling | estimated 30.64 ticks/rev from a simulated tachometer (truth 30.8), flagged the >5 % mismatch with the configured 28 |
| flywheel-disturbance (200 ms cut) | drop ~996 ticks/s (simulated SDK model), ready lost at 0.02 s, recovered 0.95 s, ready again 1.13 s |
| turret-direction | +135 ticks, `ENCODER_SIGN=+1` |
| turret-ticksperrev | 3032 ticks per turret rev (truth 3030), spread 0 %, external ratio 3.032 |
| turret-limits | stops -97.4 / +91.7 deg measured, proposed limits pulled in 5 deg |
| turret-step (20 deg, 3 V cap) | rise 0.26 s, overshoot 0.2 %, settling 0.37 s, saturation 7 %, 0 limit violations |
| turret-aim (robot turned by hand, target switch) | rms error 9.4 deg *including the hand-rotation transient and the 45 deg switch*, aligned 92 %, re-aim 0.7 s (production `AimTurret` with the placeholder plant) |
| turret sysid (test) | inertia recovered to 0.05 kg m^2 (truth 0.05), replay r2 0.9999999997 |
| intake | direction ok, 3/3 transfers, jam threshold suggestion 4.5 A (1.5 x 3.0 A simulated peak) |
| stopper-timing | simulated 150 ms operator-observed -> suggestions 190 / 180 ms (20 % margin) |
| window / window-shoot | readiness delay 1.06 s; 3 shots, 3 window entries |
| loc-velframe | verdict FIELD from 177 samples (and ROBOT detected when the simulated velocity is robot-frame - tested) |
| stage 1-6 | stage 2: 3/3 shots; stages 4-6 shot while the (stub) follower reported the path still running; stage 6 ran the intake only while on the pickup path |

Stage runs report `aligned_fraction 0` by design: with `turret.HARDWARE_CONFIGURED=false` the turret is not powered and the report says so.

## 6. Current and candidate configuration
* **Active configuration of the shipped repository: none = compiled defaults (revision 0).** No `active.properties`, no candidate and no run data are committed; the demonstration above used scratch directories under `/tmp`.
* The shipped defaults are valid and every controller designs from them (tested), but the turret and flywheel-LQR interlocks are **off** and every hardware value is a labelled placeholder.
* `calibration/templates/` holds an empty valid example and checklists of what each interlock requires.

## 7. Exact commands
```
scripts/run_tests.sh [suite ...]                 # --list shows suites          ./gradlew :TeamCode:hostTests [-Psuites="a b"]
./gradlew :TeamCode:assembleDebug                ./gradlew :TeamCode:compileDebugUnitTestJavaWithJavac
scripts/sim_calibrate.sh DIR <procedure> [--candidate] [--adopt] [--sim-targets] [--set KEY=VALUE]     # SIMULATION
scripts/offline_tune.sh DIR flywheel 1800 | turret                                                      # SIMULATION, writes a candidate only
scripts/config.sh DIR show|diff|set K V|accept "note"|reject|restore REV|restore-good|good REV|history|params-md|compare A B
scripts/analyze_log.sh sysid LOG.csv | step LOG.csv TARGET BAND | show LOG.csv
scripts/pull_config.sh DIR [push]                # adb; HARDWARE data transfer (not run here)
Robot (driver station TeleOp list, group "Calibration"): CAL Flywheel / Turret / Intake / Stopper / Localization / Shooting Window / Moving Shot / Config Manager
```
