# Tests

The test sources live in the standard Gradle location `TeamCode/src/test/java/...` (they compile with
`./gradlew :TeamCode:compileDebugUnitTestJavaWithJavac`). JUnit is not in the offline Gradle cache, so the suites use a tiny runner
(`T.java`, `AllTests.java`) instead of JUnit; they run on the host JVM against the real Ivy and Pedro classes with proxy-mocked FTC hardware.

```
scripts/run_tests.sh                       # all suites
scripts/run_tests.sh turret flywheel       # chosen suites        (scripts/run_tests.sh --list)
./gradlew :TeamCode:hostTests              # same, from Gradle / Android Studio   (-Psuites="turret ivy")
```

| Suite | File(s) | Covers |
|---|---|---|
| `hardware` | `commands/HardwareConfigTests` | defaults valid, interlocks off, missing devices, preflight, safe shutdown, invalid values |
| `config` | `tuning/ConfigFrameworkTests` | registry, validator (ranges, tighten-only, interlocks), store (candidate/accept/reject/restore/stale), boot loading, hashes, docs/templates |
| `metrics` | `tuning/MetricsSysIdTests` | step/tracking/recovery metrics, logs, reports + comparison, system identification, window tracker, velocity-frame detector |
| `control` | `control/ControlTests` | angles, geometry, conversions, DLQR, turret and flywheel controllers |
| `turret-math` | `control/TurretGeometryAndControllerTests` | gear bookkeeping (no double count), motion, limits, stability, optimality, feed-forward, saturation, filter |
| `flywheel-observer` | `control/FlywheelObserverTests` | units, Kalman, feed-forward, targets, faults, battery |
| `readiness` | `control/ShootingReadinessTests` | every window condition, boundaries, gates |
| `flywheel` | `commands/FlywheelRegulatorTests` | regulator, readiness, targets, stop, LQR path, measurement |
| `shooting`, `shoot-cancel` | `ShootingStateMachineTests`, `ShootingCancelTests` | all phases, timeouts, cancel in every phase, completion policy |
| `intake` | `IntakeTests` | direction/power/duration/jam/stopper positions |
| `turret` | `TurretAimingTests` | targets API, AimTurret, switching, limits, stale/invalid |
| `movingshot`, `auto-routines` | `MovingShotTests`, `AutoRoutineTests` | concurrent composition, windows, routines |
| `validation`, `state`, `lifecycle` | `ValidationTests`, `StateSafetyTests`, `LifecycleTests` | inputs, state, ownership, transitions |
| `ivy` | `IvyTests` | scheduler semantics |
| `opmode-exit` | `OpModeLifecycleTests` | **static source check** of OpMode exit paths |
| `cal-*`, `workflow` | `*CalibrationTests`, `CalibrationWorkflowTests` | every calibration procedure in the simulator, and the full tuning workflow + CLIs |

Not covered by any test (needs the assembled robot): real motor/encoder behaviour, real servo timing, Pedro/Pinpoint accuracy, real shot dynamics, the OpModes' execution, and `RobotHardware.init` with a real `HardwareMap`.
