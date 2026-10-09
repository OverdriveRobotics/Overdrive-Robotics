# Configuration management

**What it does.** Holds the versioned set of parameter *overrides* (`active.properties`, `candidate.properties`, `history/rev-NNNN`,
`known-good.txt`, `accepted.log`, `rejected/`, `runs/`). Files are sparse: only values that differ from the compiled defaults are stored,
so "explicitly calibrated" == "present in the file". `RobotHardware.init` applies the active file; an invalid file is ignored **as a whole**
(defaults are used and `CONFIG PROBLEMS` shows in telemetry).

**Run its tests.** `scripts/run_tests.sh config metrics workflow hardware`

**Tools.** OpMode `CAL Config Manager` (robot) - `scripts/config.sh DIR <command>` (PC; pull the folder first with `scripts/pull_config.sh DIR`).
Commands: `show candidate diff history params params-md set K V set-measured K V unset K accept [note] reject restore REV good REV restore-good compare A B`.

**Equipment.** Robot controller + (for the PC tools) `adb`. No special hardware.

**Adjustable parameters.** Everything in [`docs/PARAMETERS.md`](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/docs/PARAMETERS.md) marked hand-adjustable. `set` refuses safety caps; `set-measured` exists only for measured mechanical limits.

**Interpreting.** `diff` lists candidate vs active. `accept` re-validates ranges, tighten-only rules, interlock prerequisites and that both controllers can be designed; any failure leaves everything unchanged and prints the reasons.

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

**Safety.** Accepting never moves anything. Interlocks (`turret.HARDWARE_CONFIGURED`, `flywheel.MODEL_IDENTIFIED`) change what the robot will drive on the next OpMode init: accept them only after the validation runs in the subsystem READMEs.
