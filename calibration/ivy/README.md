# Ivy scheduler and command coordination

Ivy has no physical parameters to calibrate. What exists instead:

* **Automated tests** (hardware-independent, real scheduler 1.1.1): `scripts/run_tests.sh ivy lifecycle auto-routines shoot-cancel movingshot opmode-exit` - sequential, parallel, race, deadline, requirements, OVERRIDE/CANCEL/QUEUE, priorities, blocked commands, SUSPEND vs END, nested scheduling, timeouts, group cancellation, and that `Scheduler.reset()` does **not** call `end()`.
* **Working example routines**: `commands/AutoRoutines` (`shootWhileDriving`, `cycle`), `auto/MovingShotAutoExample` (disabled; placeholder poses), and the calibration stages that exercise them.
* **Pattern guide**: [`docs/IVY_GUIDE.md`](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/docs/IVY_GUIDE.md).

Nothing to save or restore here. OpMode exit paths are checked statically (`opmode-exit`): every OpMode must call `Scheduler.reset()` then `robot.safeShutdown(...)` - this is a source check, not an execution on the robot.
