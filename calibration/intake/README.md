# Intake and transfer calibration

**What it does.** `RunIntake` drives `intakeAndTransferMotor` (forward = intake, reverse = out), stops it on completion, failure, interruption and cancellation, and has an optional current-based jam detector (`robot.DETECT_INTAKE_JAM`, **off** by default). Shooting owns the same motor (priority 1 > intake 0), so the intake cannot run during a shot.

**Run its tests.** `scripts/run_tests.sh intake cal-intake-stopper shooting` - simulation: `scripts/sim_calibrate.sh DIR intake`.

**Run the tool.** TeleOp **CAL Intake**: forward run -> *"did it pull balls IN?"*, reverse run -> *"did it push OUT?"*, then N transfer cycles, each judged by you. Records motor current and velocity.

**Equipment.** Balls, clear hands. Current sensing must be available for jam thresholds (otherwise the tool says so).

**Adjustable parameters.** `robot.INTAKE_POWER`, `FEED_POWER`, `DETECT_INTAKE_JAM`, `INTAKE_JAM_AMPS`, `INTAKE_JAM_MS`.

**Metrics.** `direction_ok` (must be 1), `transfer_success_rate`, `current_mean_a`, `current_peak_a`, `velocity_mean_abs`. Suggestion: `INTAKE_JAM_AMPS` = 1.5 x the highest *normal* current seen. Wrong direction is reported as a wiring fact (`RobotHardware` motor direction), not a tunable.

## Save, reject, restore
Nothing is saved by running a procedure. Edits go to the **candidate** (shown next to the active value in the PARAMETERS pane).

| Action | On the robot (OpMode: press **right stick**) | On the PC (`scripts/config.sh DIR ...`) |
|---|---|---|
| Keep for later | *Save candidate to file* | `set KEY VALUE` (writes the candidate) |
| **Accept** (becomes active) | *ACCEPT candidate* + confirmation | `accept "note"` (re-runs the config tests automatically) |
| **Reject** | *REJECT candidate* + confirmation | `reject` (a copy is kept in `rejected/`) |
| Mark verified | *Mark ACTIVE revision known-good* | `good REV` |
| **Restore** | *Restore latest known-good* | `restore REV` or `restore-good` (becomes a NEW revision; history is never rewritten) |

Every report and CSV log records the configuration hash it was produced with, so a result can always be traced to its parameters.

**Safety.** Calibration power capped at 60 %; the motor is stopped by the command's `end()` on every exit path (tested: cancel, abort, completion).
