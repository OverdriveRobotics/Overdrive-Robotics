# Moving-shot staged validation

**What it does.** Runs the production `RobotCommands.movingShot` / `AutoRoutines` while recording pose, speed, window, turret error, flywheel and shot events.

**Run its tests.** `scripts/run_tests.sh movingshot auto-routines cal-window-movingshot shooting` - simulation: `scripts/sim_calibrate.sh DIR stage1` ... `stage6`.

**Run the tool.** TeleOp **CAL Moving Shot**. First set the test path in the PARAMETERS pane (`cal.PATH_START_X/Y/HEADING_DEG`, `cal.PATH_END_*`) and target A - they are unset on purpose and the stages refuse to drive until you provide them. Stages, in order - do not skip:

| Stage | What runs | Robot moves? |
|---|---|---|
| 1 | flywheel spin-up + turret aiming, no feeding | no |
| 2 | stationary shooting of `cal.STAGE_BALLS` | no |
| 3 | path following while the flywheel spins up (no feeding) | **yes** |
| 4 | one moving shot | **yes** |
| 5 | multi-ball moving shot | **yes** |
| 6 | full scoring cycle: pickup path with intake, then shoot while driving back | **yes** |

Stage 3-6 prompt: *"The ROBOT WILL DRIVE ... A=go"*; **BACK aborts**: commands are cancelled, mechanisms made safe and the drivetrain told to hold position.

**Equipment.** Cleared space >= path length + 1 m, taped start/end marks, balls, spotter on the stop button. Pedro Foresight tuned (`pedro/Constants.java` says it is not yet).

**Adjustable parameters.** `cal.PATH_*`, `cal.STAGE_BALLS`, `cal.WINDOW_MS`, `cal.STAGE_SECONDS`, plus the shooting-window parameters (see that README) and `robot.FLYWHEEL_VELOCITY_A`.

**Metrics.** `shots_ok`, `shot_success_rate`, `first_shot_after_s`, `completed_normally`, `path_completed`, `max_speed_in_s`, `aligned_fraction`, `turret_err_max_deg`, window metrics; notes list blocking reasons and failures. Compare stage N+1 only after stage N is clean.

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

**Safety.** The robot drives: keep spectators clear, use low-speed paths first. A stage is aborted by BACK, by timeout, and on any preflight failure before motion.
