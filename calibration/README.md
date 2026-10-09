# Calibration, tuning and test framework

Everything here operates on the **same parameter definitions and controller classes the robot runs** (`tuning/ParamRegistry`,
`control/*`, `commands/*`). Tuning never needs a production source edit: values live in a versioned configuration file on the
robot controller (`/sdcard/FIRST/overdrive/`), loaded by `RobotHardware.init`.

## What runs where (labels you will see in every report)

| Label | Meaning | Examples |
|---|---|---|
| **HARDWARE-INDEPENDENT** | Host JVM, no robot, real Ivy scheduler, mocked hardware, fake clock. Result = *the code behaves as specified*. | `scripts/run_tests.sh`, `./gradlew :TeamCode:hostTests` |
| **SIMULATION** | A calibration procedure run against the simulator. Exercises the tooling and the controllers; says nothing about the real robot. Reports are marked `SIMULATION`. | `scripts/sim_calibrate.sh`, `scripts/offline_tune.sh` |
| **HARDWARE** | Guided OpModes on the assembled robot ("CAL ..." in the TeleOp list). Reports are marked `HARDWARE`. | `CAL Flywheel`, `CAL Turret`, ... |

Ordinary unit tests never touch real hardware and never write the robot configuration.

## The workflow (same for every subsystem)
1. **Load** the active configuration (automatic) and check preflight (devices present, battery >= 10.5 V, no shot in progress).
2. **Run** a procedure. It logs timestamped data (CSV) and computes subsystem metrics.
3. **Read** the metrics and the *comparison with the first run* (better / WORSE / same, with the parameter changes listed).
4. **Adjust** supported parameters in the PARAMETERS pane (candidate only) - or press **Y** to adopt the procedure's measured suggestions.
5. **Rerun** with **A** (candidate) - **X** reruns with the active settings for a fair comparison.
6. **Accept or reject** explicitly (right stick -> configuration menu). Accepting re-validates everything first.
7. **Re-run the automated tests** after accepting (`scripts/config.sh ... accept` does it for you on the PC).

Safety rules enforced in code: operator confirmation before any motion, **BACK aborts any procedure**, conservative caps
(`calibration/CalConfig`: 3 V turret, 70 % open-loop flywheel, 60 % intake), limits respected, mechanisms returned to their safe
state on completion, abort *and* failure, and `Scheduler.reset()` is never trusted to stop hardware.

## Parameter categories
Fixed/derived (never edited by tools) - hardware calibration (measured) - controller tuning - behavioural thresholds - **safety limits**
(not adjustable by the interactive tools; a file may only *tighten* them; mechanical limits are measured-only) - **interlocks**
(`turret.HARDWARE_CONFIGURED`, `flywheel.MODEL_IDENTIFIED`, `robot.REQUIRE_TURRET_ALIGNED`): the validator refuses to enable one
until every value it depends on has been **explicitly** calibrated, never inherited from a placeholder default.
Full table: [`docs/PARAMETERS.md`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/docs/PARAMETERS.md).

## Layout
| Path | Contents |
|---|---|
| `TeamCode/src/main/.../tuning/` | registry, validator, config store, metrics, reports, system identification, offline tuner, CLIs |
| `TeamCode/src/main/.../calibration/` | procedures (shared by robot and simulation), session/editor, `opmodes/` (the guided OpModes) |
| `TeamCode/src/test/.../` | all hardware-independent tests + the simulator (`SimRobot`, `SimCalContext`, `SimCalibrate`) |
| `scripts/` | `run_tests.sh`, `sim_calibrate.sh`, `config.sh`, `offline_tune.sh`, `analyze_log.sh`, `pull_config.sh`, `gen_param_docs.sh` |
| `calibration/<subsystem>/README.md` | per-subsystem instructions (this folder) |
| `calibration/templates/` | example configuration + checklists |
| `docs/` | `CALIBRATION_GUIDE.md` (order of operations), `IVY_GUIDE.md`, `PARAMETERS.md`, `SUBSYSTEM_INVENTORY.md`, `TEST_REPORT.md`, `MOVING_SHOT_SYSTEM.md` |
| `tests/README.md` | map of test suites -> files -> commands |

Start with [`docs/CALIBRATION_GUIDE.md`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/docs/CALIBRATION_GUIDE.md).
