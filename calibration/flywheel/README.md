# Flywheel calibration

**What it does.** The flywheel regulator (`FlywheelRegulator`) is the only writer of the shooter motors and the only owner of flywheel readiness.
Default path: the SDK's velocity PIDF. After identification (`flywheel.MODEL_IDENTIFIED=1`): a scalar state-space model + Kalman observer + LQR +
plant-inversion feed-forward, output in volts converted with live battery voltage (`FlywheelLqrController`).

**Run its tests.** `scripts/run_tests.sh flywheel flywheel-observer cal-flywheel control` (hardware-independent) -
`scripts/sim_calibrate.sh DIR flywheel-step | flywheel-sysid | flywheel-disturbance | flywheel-scaling` (simulation).

**Run the tool.** TeleOp **CAL Flywheel**:
1. *Encoder scaling check* - spins at 30 % and asks for a tachometer reading of the **flywheel** rpm; estimates ticks/rev.
2. *Open-loop system identification* - 35/50/65 % power steps (3 s each) + 4 s coast-down; fits kS/kV/kA by least squares.
3. *Step response* - drives the **production regulator** to `robot.FLYWHEEL_VELOCITY_A`.
4. *Disturbance recovery* - the regulator cuts power for `cal.DISTURBANCE_MS` (a controlled stand-in for a ball; not a real ball).

**Equipment.** Flywheel guard fitted, area clear, battery >= 10.5 V, a tachometer (for #1), no balls loaded.

**Adjustable parameters.** `robot.FLYWHEEL_TOLERANCE`, `FLYWHEEL_STABLE_MS`, `FLYWHEEL_SPINUP_TIMEOUT_MS`, `FLYWHEEL_VELOCITY_A/B`; `flywheel.MAX_VELOCITY_ERROR` (LQR weight), `MODEL_STD`, `MEASUREMENT_STD` (observer), `KS_VOLTS`, `KV_VOLTS_PER_TICK_S`, `KA_VOLTS_PER_TICK_S2`, `USE_SYSID_MODEL`, `SHAFT_TICKS_PER_REV`; interlock `MODEL_IDENTIFIED`. Caps (`MAX_CONTROL_VOLTS`, `MAX_PLAUSIBLE_VELOCITY`, `robot.MAX_FLYWHEEL_VELOCITY`) are not adjustable.

**Metrics and how to read them.**
| Metric | Meaning | Good looks like |
|---|---|---|
| `settling_time_s`, `rise_time_s`, `overshoot_pct` | step response to the target | short, small overshoot |
| `steady_state_error` | mean(measured - target) over the last 25 % | inside `FLYWHEEL_TOLERANCE` |
| `readiness_delay_s` | time until the regulator declares ready (settled + `FLYWHEEL_STABLE_MS`) | settling + stable window |
| `saturation_fraction` | fraction of samples at power >= 0.999 | low (high = under-powered / aggressive weights) |
| `power_range_violations` | samples outside [0,1] | **0** |
| `battery_min_v` | sag during the run | stays above ~10.5 V |
| `disturbance_drop`, `recovery_time_s`, `ready_again_after_s` | disturbance response | recovery well below the time between shots |
| `fit_kV/kA/kS`, `fit_r2`, `fit_rmse` | identification; r2 is a **replay** of the fitted model over the recorded voltages | r2 > ~0.9, else repeat |

**Recommended order.** scaling -> sysid -> **Y** (adopt suggestions) -> set `MODEL_IDENTIFIED` in the candidate -> step response (**A**) and disturbance (**A**) under the candidate -> compare with the **X** (active/SDK-PIDF) runs -> accept only if it is better *on the robot*. Tune `MAX_VELOCITY_ERROR`/`MODEL_STD` one at a time; `scripts/offline_tune.sh DIR flywheel 1800` can propose a starting point (simulation).

**Honest status.** In tests the LQR path equals a feed-forward + P controller with the same gain (one state => LQR is a proportional gain). Whether it beats the SDK PIDF on the real wheel is **unmeasured**; the simulator's "SDK PIDF" is an arbitrary first-order model, so simulated LQR-vs-SDK comparisons say nothing about the robot.

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

**Safety.** Confirmation prompt before open-loop runs; power capped at 70 %; abort/low battery/failure => power 0 and the regulator target cleared; motors handed back in `RUN_USING_ENCODER`. Never run with a ball in the transfer.
