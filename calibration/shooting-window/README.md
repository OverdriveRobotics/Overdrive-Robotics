# Shooting-window calibration

**What it does.** `ShootStatus` / `ShootingReadiness` decide, from live data every loop, whether a shot is allowed: localization fresh, inside the zone rectangle, speed limit, target valid, turret sensor fresh + reachable + aligned, flywheel stable and fresh, no faults, feed free. The first failing reason is always reported.

**Run its tests.** `scripts/run_tests.sh readiness shooting shoot-cancel cal-window-movingshot state` - simulation: `scripts/sim_calibrate.sh DIR window | window-shoot`.

**Run the tool.** TeleOp **CAL Shooting Window**: *monitor only* (never feeds) or *monitor + shoot N* (the shot waits for the window). Push the robot through the zone to see it open and close.

**Equipment.** Robot, spare balls (for shoot runs), measured zone corners, flywheel guard.

**Adjustable parameters.** `robot.REQUIRE_LOCALIZATION`, `SHOOT_ZONE_ENABLED`, `SHOOT_ZONE_MIN/MAX_X/Y`, `MAX_SHOOT_SPEED`, `REQUIRE_TURRET_ALIGNED` (interlock), `turret.ALIGN_TOLERANCE_RAD`, `ALIGN_VELOCITY_TOL_RAD_S`, `MAX_POSE_AGE_MS`, flywheel tolerance/stability, `READY_TIMEOUT_MS`, `SHOT_TIMEOUT_MS`, `RECOVER_TIMEOUT_MS`, `SHOT_DROP_THRESHOLD`, `FLYWHEEL_VELOCITY_A/B`.

**Metrics.** `window_entries`, `time_in_window_s`, `time_outside_window_s`, `longest_window_s`, `readiness_delay_s`, `shots_ok`, `shot_success_rate`, `first_shot_after_s`, and *notes* of the form `blocked 2.31 s: outside shooting zone` (what prevented shooting, and for how long).
Read: if `longest_window_s` is shorter than open-settle + feed + detection (~0.5-1 s), the window is too small for a moving shot - widen the zone, lower `MAX_SHOOT_SPEED`, or relax the alignment tolerance, then rerun and compare.

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

**Safety.** Shoot runs need your confirmation; the mechanism is closed/stopped and the flywheel stopped afterwards; leaving the zone mid-ball does not abort the ball (it would risk a jam) but no further ball starts.
