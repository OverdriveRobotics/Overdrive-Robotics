# Ivy on this robot - a short guide for new members

Ivy runs **commands** cooperatively on one thread: every loop you call `Scheduler.execute()` once; each running command gets one `execute()`.
A command has `start()`, `execute()`, `done()`, `end(EndCondition)`. `EndCondition` is `NATURALLY`, `INTERRUPTED` or `SUSPENDED`.
(All behaviour below is verified by `scripts/run_tests.sh ivy`.)

## The three rules of this project
1. **Requirements = ownership.** A command that *writes* a device must require it (`Resources.flywheel/stopper/feed/turret`). Commands with different requirements run side by side; with the same requirement they conflict.
2. **`Scheduler.reset()` does not stop hardware.** It forgets commands without calling `end()`. Every OpMode exit does `Scheduler.reset(); robot.safeShutdown(...)`.
3. **Only the flywheel regulator writes the shooter motors.** Spin-up/shoot commands *request* a target; `StopFlywheel` is the one command allowed to override the regulator.

## Priorities and conflicts (what the robot uses)
| Command | Priority | Conflict | Why |
|---|---|---|---|
| `StopFlywheel` | 10 | OVERRIDE | always wins |
| `FlywheelRegulator` | 5 | CANCEL | a second one cannot displace the running one |
| `ShootBalls` | 1 | CANCEL | beats the intake, cannot be displaced by it |
| `RunIntake`, `AimTurret`, `SpinUpFlywheel` | 0 | OVERRIDE | newest request wins |

`InterruptedBehavior` is `END` for every robot command: a half-finished feed is never resumed.

## Building routines
```java
import static com.pedropathing.ivy.groups.Groups.*;           // sequential, parallel, race, deadline
import static com.pedropathing.ivy.pedro.PedroCommands.*;     // follow(follower, path), hold(follower)

Scheduler.schedule(sequential(
    deadline(follow(f, toPickup), RobotCommands.runIntake(robot, false)),   // intake ONLY while driving; path = deadline
    RobotCommands.movingShot(robot, toShoot, 3, 1800, 5000),               // = parallel(follow, shoot): shooting does not wait for the path
    hold(f)));
```
* `sequential(a, b)` one after another - `parallel(a, b)` together, ends when **all** end - `race(a, b)` ends with the **first** (others are interrupted) - `deadline(a, b...)` ends when `a` ends.
* `cmd.until(() -> cond)`, `waitUntil(cond)`, `waitMs(ms)`; `race(x, waitMs(2000))` is a timeout.
* A group requires the union of its children's requirements and conflicts as a unit.
* Pedro's `follow()` has **no** requirements and does **not** call `follower.update()`: your loop must call `robot.updateLocalization()`.
* Create a **new** command object for every schedule (the `RobotCommands` factories do).

## Background loops
`RobotCommands.startShooterSystems(robot)` once per OpMode (after `Scheduler.reset()`): flywheel regulator + `AimTurret` + readiness monitor. They own distinct resources so they run alongside driving and shooting.

## A minimal OpMode skeleton
```java
init():  robot.init(hardwareMap, x, y, heading); Scheduler.reset();
start(): RobotCommands.startShooterSystems(robot); Scheduler.schedule(myRoutine);
loop():  robot.updateLocalization(); Scheduler.execute(); RobotTelemetry.add(telemetry, robot); telemetry.update();
stop():  Scheduler.reset(); robot.safeShutdown(true);
```
Examples: `auto/MovingShotAutoExample`, `teleop/blue/blueTele`, `commands/AutoRoutines`. Test a routine on the host by driving it with `SimRobot` (see `AutoRoutineTests`).
