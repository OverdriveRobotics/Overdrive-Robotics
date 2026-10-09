# Stopper servo calibration

**What it does.** The ball gate (`stoper` servo). `ShootBalls` opens it, waits `robot.STOPPER_SETTLE_MS`, feeds, closes it, waits `STOPPER_CLOSE_SETTLE_MS`. **There is no position feedback**, so the delays are assumptions you calibrate by observation; a commanded position never proves the gate moved.

**Run its tests.** `scripts/run_tests.sh intake cal-intake-stopper shoot-cancel shooting` - simulation: `scripts/sim_calibrate.sh DIR stopper-timing | stopper-endpoints`.

**Run the tool.** TeleOp **CAL Stopper**: (1) *Jog + set positions* - the servo moves **slowly** (<= 0.5 units/s) while you confirm or re-enter each endpoint; (2) *Operator-timed delays* - the servo snaps open/closed as in a match and you press **A** the instant it has fully settled, 5 repeats.

**Equipment.** Gate clear of balls and fingers; ideally a phone video at 60 fps to cross-check your timing.

**Adjustable parameters.** `robot.STOPPER_CLOSED`, `STOPPER_OPEN`, `STOPPER_SETTLE_MS`, `STOPPER_CLOSE_SETTLE_MS` (open and closed must differ).

**Metrics.** `open_mean/max/stddev_ms`, `close_mean/max/stddev_ms` (include your reaction time, ~150-250 ms), suggestion = max x 1.2 rounded up to 10 ms. These are *upper bounds*, deliberately conservative.

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

**Safety.** The servo is returned to CLOSED on completion, abort and failure. Endpoint jogging never jumps.
