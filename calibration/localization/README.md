# Localization and coordinate-frame calibration

**What it does.** Pedro's `Follower.pose()` (inches, heading radians CCW from +x) feeds aiming and the shooting window; `Follower.velocity()` feeds the turret's motion feed-forward.
**Verified in software:** the installed Pedro treats `velocity()` as **field-frame** (`PinpointLocalizer` -> `MotionState.ofVelocity(pose, velocity)`; the robot-frame `Twist` is derived from it) and `TurretUtil`'s frame conversion matches Pedro's own `Twist.toVelocity` (tested). **Not verifiable in software:** the raw Pinpoint output - procedure 1 checks it on the robot.

**Run its tests.** `scripts/run_tests.sh cal-localization turret-math readiness` - simulation: `scripts/sim_calibrate.sh DIR loc-velframe`.

**Run the tool.** TeleOp **CAL Localization** (the drivetrain is never powered - push the robot by hand):
1. *Velocity coordinate frame* - rotate to ~90 deg, push around for 15 s; compares reported velocity with pose-derived velocity; verdict FIELD / ROBOT / undetermined.
2. *Heading: one full revolution* - accumulated heading vs 360 deg.
3. *Straight distance vs tape* - scale error.
4. *Reference poses* - you enter 3 taped poses and place the robot on them.
5. *Measure target points A and B* - stand the robot centre over each aim point; the pose becomes `target.A_X/A_Y/B_X/B_Y`.

**Equipment.** Tape measure, floor marks, the robot with the odometry installed.

**Adjustable parameters.** `turret.ROBOT_VELOCITY_IS_FIELD_FRAME`, `target.A_X/A_Y/B_X/B_Y`.

**Metrics.** `pos_error_mean/max_in`, `heading_error_max_deg`, `revolution_error_pct`, `scale_error_pct`, `verdict_field_1_robot_2_unknown_0`, `samples`. Scale and heading errors are fixed in the Pinpoint/Pedro odometry tuning (`pedro/Constants.java`, Pedro tuners), **not** in these parameters; this tool tells you whether they need it.

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

**Safety.** No actuator is commanded. Targets must be measured in the same field origin the autonomous initialises to.
