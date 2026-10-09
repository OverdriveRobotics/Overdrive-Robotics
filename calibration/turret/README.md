# Turret calibration

**What it does.** `AimTurret` points the turret at the active field target using the Pedro pose; `TurretStateSpaceController` (state `[angle, velocity]`,
volts in, LQR + plant-inversion feed-forward, slewed reference) drives it. The motor is **never powered by production code** until `turret.HARDWARE_CONFIGURED=1` is accepted.

**Run its tests.** `scripts/run_tests.sh turret turret-math cal-turret control readiness` - simulation: `scripts/sim_calibrate.sh DIR turret-step | turret-aim | turret-direction | turret-ticksperrev | turret-limits`.

**Run the tool.** TeleOp **CAL Turret** (turret centred on its forward mark *before* pressing INIT):
1. *Encoder direction* - 0.3 s nudge at <= 0.15 power; reports the sign, or tells you to reverse the motor in `RobotHardware` (not tunable).
2. *Ticks per turret revolution* - **motor unpowered**; you rotate it by hand by a known angle, N repeats. Gives `SHAFT_REVS_PER_TURRET_REV` and the encoder consistency (spread %).
3. *Mechanical limits* - motor unpowered; push to each hard stop; proposes limits pulled in by 5 deg.
4. *Zero check* - how far the forward mark is from 0 deg (fix by centring before INIT, not by editing numbers).
5. *Pulse system identification* - +-0.5/0.75/1x 1.5 V pulses; fits the plant, back-computes inertia/friction **using the datasheet constants you entered**.
6. *Step response* - +-`cal.TURRET_STEP_DEG` with the production controller at <= 3 V.
7. *Aim accuracy* - runs the production `AimTurret`; **turn the robot by hand** to test heading compensation; switches target mid-run; asks you for the observed aim offset.

**Equipment.** Robot on blocks or in a clear area, hand access to the turret, a protractor/marks for the known angle, target points A/B measured (CAL Localization #5).

**Gearing rule (no double counting).** `turret.SHAFT_TICKS_PER_REV` is the encoder resolution of **the same shaft the motor datasheet numbers describe** (including its internal gearbox); `turret.SHAFT_REVS_PER_TURRET_REV` is the **external** reduction only. Procedure 2 computes `ratio = measured ticks per turret rev / SHAFT_TICKS_PER_REV`, so `shaftTicks x ratio` always equals what you measured.

**Adjustable parameters.** Hardware: `SHAFT_TICKS_PER_REV`, `SHAFT_REVS_PER_TURRET_REV`, `ENCODER_SIGN`, `START_ANGLE_RAD`, motor datasheet values, `LOAD_INERTIA_KG_M2`, `VISCOUS_FRICTION`, `STATIC_FRICTION_VOLTS`, targets A/B. Controller: `MAX_ANGLE_ERROR_RAD`, `MAX_VELOCITY_ERROR_RAD_S` (LQR weights), `MAX_REFERENCE_VELOCITY_RAD_S`, `VELOCITY_FILTER_ALPHA`, `INTEGRAL_GAIN`. Behaviour: `ALIGN_TOLERANCE_RAD`, `ALIGN_VELOCITY_TOL_RAD_S`. Limits `MIN/MAX_ANGLE_RAD` are **measured-only**; voltage cap and limit margin are not adjustable.

**Metrics.** `delta_ticks`, `ticks_per_turret_rev(+stddev, encoder_spread_pct)`, `configured_vs_measured_pct`, `range_deg`, `zero_error_deg`; step: `rise_time_s`, `overshoot_pct`, `settling_time_s`, `tracking_rms/max`, `saturation_fraction`, `peak_power`, `limit_violations`; aim: `aim_rms_deg`, `aim_max_deg`, `aligned_fraction`, `realign_time_s`, `fault_samples`, `observed_aim_offset_deg`.
Read: spread < 1 % means a trustworthy ratio; `limit_violations` must be 0; `observed_aim_offset_deg` is the only number that measures the *real* aim.

**Order.** 1 -> 2 -> 3 -> (datasheet values) -> 5 -> adopt (**Y**) -> set `turret.HARDWARE_CONFIGURED=1` in the candidate (the validator lists anything still missing) -> 6 -> 7 -> accept.

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

**Safety.** Turret voltage capped at 3 V (`CalConfig`); limits respected (power into a limit is cut; leaving the range aborts); manual steps leave the motor unpowered; placeholder values trigger an explicit warning prompt; abort/failure => power 0. Keep hands and cables clear.
