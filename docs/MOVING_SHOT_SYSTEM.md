# Moving-shot system: design, status, calibration

Status: **implemented, compiled, and verified in simulation only.** Nothing here has run on the robot. The
turret and flywheel models use **placeholder** hardware numbers (marked `PLACEHOLDER` in the config files);
both state-space paths are interlocked **off** until those are measured.

Run the tests: `scripts/run_tests.sh` (host JVM, real Ivy scheduler, proxy-mocked hardware, fake clock).
JUnit is not in the offline Gradle cache, so the tests use a small runner (`T.java`). Main code:
`./gradlew :TeamCode:assembleDebug`; the test sources also compile under Gradle (`compileDebugUnitTestJavaWithJavac`).

## Files

New: `control/` (`AngleUtil, Matrices, Lqr, TurretConfig, TurretUtil, TurretStateSpaceController, FlywheelConfig,
FlywheelLqrController, ShootingReadiness` - pure Java, no FTC imports), `commands/` (`AimTurret, FlywheelRegulator,
ShootStatus, Resources, RobotTelemetry`), `hardware/RobotClock.java`, `auto/MovingShotAutoExample.java` (`@Disabled`),
`scripts/run_tests.sh`, `TeamCode/src/test/...`.
Modified: `Storage, RobotHardware, RobotConfig, RobotCommands, BaseCommand, FlywheelUtil, SpinUpFlywheel,
StopFlywheel, ShootBalls, RunIntake, blueTele`. (Removed one unused `import com.pedropathing.*;` from RobotHardware.)

## What was found in the project

* No autonomous OpModes existed; only `blueTele`, which never used the Ivy scheduler. There was no scheduler init.
* `datar.txt` does not exist. The files are `data/datax.txt` (WPILib *state-space flywheel* tutorial) and
  `data/doc.txt` (Ivy docs). Both were read; see "How the data files were used".
* Turret: `DcMotorEx "turret"`, **relative** encoder (zero = position at init), motor type / gearing unknown.
  Shooter: `shooterLeft/Right` `DcMotorEx`, velocity in ticks/s, motor type unknown. Feed: `intakeAndTransferMotor`.
  Stopper: servo `stoper`, no position feedback. No ball sensor. No turret absolute encoder.
* Pose: Pedro `follower.pose()` (`x, y` in inches, `heading` in radians, CCW from +x) plus `follower.velocity()`.
  Pedro's `follow()` command does **not** call `follower.update()` and has no requirements.
* `RobotHardware` set `PIDFCoefficients(1,1,1,1)` on the shooters and turret. These are placeholders that were
  already there; the flywheel's SDK-PIDF path (the default) still depends on them. They are untouched and should be tuned.
* Java source level 8; Ivy 1.1.1; Pedro core 3.0.1.

## Ivy behaviour relied on (verified by tests against the real scheduler)

| Behaviour | Verified |
|---|---|
| Equal priority + `ConflictBehavior.CANCEL`: the second command is not scheduled, the first keeps running | second `ShootBalls` |
| Higher priority interrupts lower; `InterruptedBehavior.END` calls `end(INTERRUPTED)` | shot interrupts `RunIntake` |
| Lower priority is blocked (`BlockedBehavior.CANCEL`) | intake scheduled while shooting is rejected |
| `Scheduler.reset()` does **not** call `end()` and does not touch hardware | feed motor stays on after `reset()`; hence `RobotHardware.safeShutdown()` |
| `Scheduler.schedule()` from inside another command's `start()` works | `SpinUpFlywheel` -> regulator |
| `parallel(a, b)` ends when both end; children run concurrently | `movingShot` |
| `Scheduler.cancel()` calls `end(INTERRUPTED)` | cancel in OPENING / FEED |

## Resource ownership

| Command | Requires | Priority / conflict |
|---|---|---|
| `FlywheelRegulator` (internal, only shooter-motor writer) | flywheel | 5 / CANCEL |
| `StopFlywheel` | flywheel | 10 / OVERRIDE (interrupts the regulator) |
| `SpinUpFlywheel` | none (sets target, waits for measured readiness) | 0 |
| `AimTurret` | turret | 0 / OVERRIDE |
| `ShootBalls` | stopper + feed motor | 1 / CANCEL |
| `RunIntake` | feed motor | 0 / OVERRIDE |
| readiness monitor, Pedro `follow` | none | - |

Driving, aiming, flywheel and shooting therefore never serialise each other; intake cannot run during a shot.

## Turret model (`control/TurretStateSpaceController`)

State `x = [theta (rad), omega (rad/s)]`, input = motor **volts**, output `y = theta`.
Continuous: `theta' = omega`, `omega' = -a*omega + b*V`, with
`a = (G^2*Kt*Kv/R + bf)/J`, `b = G*Kt/(R*J)`, `R = Vnom/Istall`, `Kt = Tstall/Istall`, `Kv = Vnom/wfree`,
`G` = shaft revs per turret rev. Discretised with an exact zero-order hold (matrix exponential) at `T = 20 ms`:
`x[k+1] = Ad x[k] + Bd u[k]`. Gain by discrete LQR (DARE iteration, once at construction) with Bryson weights
`Q = diag(1/(5 deg)^2, 1/(1 rad/s)^2)`, `R = 1/(12 V)^2`.

Control law: `u = Bd+ (r[k+1] - Ad r[k]) + K (r[k] - x[k])` (plant-inversion feedforward + LQR), reference `r` =
slew-limited angle (6 rad/s, so a target switch is a bounded move) with the reference velocity from target motion,
optional kS and anti-windup integral (off by default), output clamped to `min(12 V, battery)` and divided by the
live battery voltage to get FTC power. Velocity: SDK `getVelocity()` through a first-order filter.

With the *placeholder* parameters: `a = 68`, `b = 37.5`, `K = [24.57 V/rad, 0.80 V/(rad/s)]`. **These numbers
are meaningless for the real turret**; they only exercise the code.

**Gear/encoder chain (no double counting):** the motor figures (`SHAFT_FREE_SPEED`, `SHAFT_STALL_TORQUE`,
`SHAFT_TICKS_PER_REV`) all describe the *same shaft* (motor + its internal gearbox). `SHAFT_REVS_PER_TURRET_REV`
is only the *external* reduction. `theta = start + sign * (ticks - zero) / ticksPerRev / G * 2pi`.

**Azimuth:** `bearing = atan2(yt - yr, xt - xr)`; `relative = wrap(bearing - heading)`; then the equivalent
`relative + 2*pi*k` that lies in `[MIN_ANGLE, MAX_ANGLE]` and is nearest the measured angle (or, if `CONTINUOUS`,
the shortest path). No equivalent in range -> target flagged **unreachable**, no readiness, turret holds. Power
into a hard limit is cut within `LIMIT_MARGIN_RAD`. The desired-angle rate from robot motion
`(dy*vx - dx*vy)/(dx^2+dy^2) - omega` is fed forward (verified against a finite difference). ASSUMPTION:
`follower.velocity()` is in the field frame (`TurretConfig.ROBOT_VELOCITY_IS_FIELD_FRAME`).

**Targets:** `Storage.targetA / targetB` (`FieldPoint`, NaN = unset), `Storage.activeTargetIndex`,
`Storage.setTarget(i,x,y)`, `selectTarget(i)`, `toggleTarget()`. Invalid indices/coordinates are rejected and leave
state unchanged. `AimTurret` re-reads the active target every loop, so switching needs no restart. Targets and the
selection survive OpMode transitions on purpose (they are configuration). **Target coordinates are unset - fill in
real field points.**

## Flywheel model (`control/FlywheelLqrController`, `commands/FlywheelRegulator`)

State `omega` in **ticks/s** (what `getVelocity()` returns), input volts: `omega' = -a*omega + b*V`,
`a = (N*Kt*Kv/R + bf/G^2)/(J/G^2)`, `b = (N*Kt/(R*J/G^2)) * ticksPerRev/(2pi)` (physical path), or from system
identification `a = kV/kA`, `b = 1/kA` (preferred once measured). Discretised as `Ad = e^(-aT)`,
`Bd = b(1 - Ad)/a` (recomputed from the measured `dt` each loop; gains designed for nominal T).
Structure from `datax.txt`: steady-state **Kalman** observer (gain from the filter Riccati iteration) +
**LQR** feedback + **plant-inversion feedforward** `(r - Ad r)/Bd (+ kS)`, output volts -> power via live battery
voltage, clamped to `[0, min(12, battery)]`. Placeholder-parameter values: `a = 1.63`, `b = 371`, `K = 0.0748 V/(tick/s)`,
`L = 0.688`. Two shooter motors get the same command and the **average** velocity is controlled (assumes both drive
one flywheel; they are not controlled independently).

**Honest assessment:** one state means LQR collapses to a proportional gain, i.e. *model feedforward + P* with the
gain chosen from `Q`/`R`. In the simulation test (`flywheel recovery after a ball`) LQR and a feedforward+P baseline
using the same gain recover identically (0.12 s from a 250 ticks/s drop): **no measured advantage**. The benefits
are that gains follow from physical parameters and that voltage compensation handles battery sag (simulated:
0.66 power at 12 V vs 0.79 at 10 V for the same speed). Whether it beats the SDK's PIDF on the real flywheel is
unknown until identified and tested.

**Regulator:** one always-on command is the only shooter-motor writer and the only owner of flywheel readiness
(`ready` = measured velocity inside tolerance continuously for `FLYWHEEL_STABLE_MS`, recomputed every loop from the
live sample; stale samples (>100 ms) count as not ready). `SpinUpFlywheel`/`ShootBalls` only *request* a target
(`FlywheelUtil.requestTarget`): an identical repeat changes nothing; a different target resets readiness and
supersedes older requests. The regulator is scheduled by `startShooterSystems` and, as a fallback, by spin-up/shoot.
`MODEL_IDENTIFIED=false` (default) keeps the project's previous SDK `setVelocity` path.

## Readiness (`control/ShootingReadiness` + `commands/ShootStatus`)

Evaluated from live data on every call; first failing reason is reported (`Storage.notReadyReason`). Conditions:
flywheel fault; (if enabled) localization finite and fresh (<100 ms since `updateLocalization()`), inside the
rectangular shooting zone, speed below `MAX_SHOOT_SPEED`, active target valid, turret sensor fresh, target
reachable, turret error < 2 deg and velocity < 0.5 rad/s, turret fault; flywheel target > 0 and stable and fresh; feed
mechanism free. The turret/zone/localization gates are switches in `RobotConfig` (`REQUIRE_TURRET_ALIGNED`,
`SHOOT_ZONE_ENABLED`, `REQUIRE_LOCALIZATION`), **default off** so existing behaviour is preserved until each is calibrated.

## ShootBalls

`WAIT_READY -> OPENING -> FEED -> CLOSING -> RECOVER -> WAIT_READY ... -> DONE`.
Readiness re-checked before every ball; stopper opens, feed starts only after `STOPPER_SETTLE_MS`; shot = RPM drop
>= `SHOT_DROP_THRESHOLD` below peak for `DROP_CONFIRM_LOOPS`; on detection the feed stops and the stopper closes at
once (closed between shots to retain the next ball); `CLOSING` settle; `RECOVER` waits on *measured* flywheel
readiness; no recovery after the last ball. **Policy:** a ball already being fed is finished even if readiness drops
mid-ball (halting risks a jam); no new ball starts until readiness is valid; leaving the zone before feeding just
waits (up to `READY_TIMEOUT_MS`) while driving continues. Timeouts: ready, shot, recover. Failure/cancel/end all
close the stopper and stop the feed; the flywheel is never stopped. A command rejected for bad parameters touches no hardware.

## Concurrent composition

```java
RobotCommands.startShooterSystems(robot);   // once, after Scheduler.reset(): regulator, aimTurret, readiness monitor
Scheduler.schedule(sequential(
    RobotCommands.movingShot(robot, path, 3, velocity, 5000),   // = parallel(follow(path), shoot(...))
    hold(robot.follower)));
// every loop: robot.updateLocalization(); Scheduler.execute();
```
Shooting does not wait for the path (tested: shots completed before the path ended; and a window that never comes
times out while the path continues). See `auto/MovingShotAutoExample.java` (disabled; placeholder poses) and `blueTele`
(bumpers: shoot / switch target; A/B spin up/stop; X/Y intake). OpMode `stop()` calls `Scheduler.reset()` then
`robot.safeShutdown(true)` because `reset()` does no cleanup.

## How the data files were used

`datax.txt`: choice of velocity as the single state and volts as the input (battery-sag compensation), the
two modelling routes (system-ID `kV/kA` and motor+inertia), the Kalman filter for lag-free velocity, LQR feedback +
plant-inversion feedforward, and the angle-wrap-with-LQR idea (done before the controller, via the reachable-angle
selection). Deliberately *not* copied: WPILib's specific numbers (kV, inertia, NEO) - they are not this robot.
`doc.txt` (Ivy): requirements, priorities, `ConflictBehavior`/`BlockedBehavior`/`InterruptedBehavior`, the command
lifecycle and `Scheduler.reset()`; behaviours were then confirmed by test.

## Tests (simulation only) - 54 pass

Control maths (20): wrap/shortest path, azimuth in all quadrants and with heading, zero/near-zero deltas, rate
feedforward vs finite difference, reachable-angle choice and rejection, continuous wrap, gear/encoder conversion and
round trip, limit guard, ZOH vs closed form, DARE vs scalar iteration, uncontrollable pair rejected, turret
step/target-switch/moving-target/invalid-input, flywheel spin-up/target change/battery sag/limits/faults/recovery.
Commands on the real Ivy scheduler (34): stability not premature; ball drop invalidates readiness next loop; repeated
and replaced targets; stop semantics; spin-up timeout; NaN velocity; LQR-path flywheel; single and 3-ball shots; no
recovery after the last ball; opening-before-feed ordering; ready / shot / recovery timeouts; cancel in OPENING and
FEED; intake conflict both directions; zone exit before and during a ball; stale/invalid localization; null robot,
missing hardware and invalid parameters; intake durations/power; targets API; AimTurret tracking, target switching,
unreachable and invalid targets, driving+turning, uncalibrated hold-off, stale pose; concurrent moving shot and
window timeout; OpMode-transition cleanup; second-shoot conflict. **Not tested:** anything on hardware; `blueTele`
and the example auto (compile only); real latencies; Pedro's real velocity frame; servo timing; real ball dynamics.

## Still to measure / decide (blocking for competition)

Turret: motor datasheet values, `SHAFT_TICKS_PER_REV`, external ratio, `ENCODER_SIGN`, start angle, mechanical
limits (inside the real hard stops), inertia, viscous + static friction; then set `HARDWARE_CONFIGURED = true`.
Flywheel: number of motors on the wheel, motor values or (better) sysid `kS/kV/kA`, inertia, friction, ticks/rev,
Kalman std devs; then set `MODEL_IDENTIFIED = true`. Field: target A/B coordinates, shooting-zone rectangle, max shot
speed, per-target velocities (`FLYWHEEL_VELOCITY_A/B` are assumptions). Mechanism: stopper open/close positions and
settle times, feed power, `SHOT_DROP_THRESHOLD`, timeouts. Pedro: `follower.velocity()` frame, and tune Foresight
(`Constants.java` says it is untuned). SDK PIDF values in `RobotHardware` (all `1,1,1,1`).

### Calibration procedures

* **Turret inertia/friction (sysid):** lifted/free turret, `RUN_WITHOUT_ENCODER`, apply a constant small voltage
  (e.g. 2 V) and log velocity: steady speed gives `kV`, the time constant gives `kA`; ramp voltage slowly until it
  just moves for `kS`. Convert to `a = kV'/kA'`, `b = 1/kA'`, or set `J, bf` so the model matches. Replay and compare.
* **Flywheel sysid:** apply steady power steps (battery voltage logged), record `omega` ticks/s: fit
  `V = kS + kV*omega + kA*omega'`, set `USE_SYSID_MODEL = true`. Check recovery after a real ball; widen `MODEL_STD` if sluggish.
* **Stopper timing:** film at 60 fps, set `STOPPER_SETTLE_MS` / `STOPPER_CLOSE_SETTLE_MS` with margin.
* **Shot detection:** log velocity during real shots; set `SHOT_DROP_THRESHOLD` well above noise and below the real drop.

### Staged physical test checklist

1. Robot on blocks, wheels free; flywheel guard on; `HARDWARE_CONFIGURED=false`, `MODEL_IDENTIFIED=false`. Run `blueTele`,
   confirm telemetry: pose updates, turret desired angle changes when switching A/B, motors idle. Turret `power` stays 0.
2. Turret unpowered: rotate by hand, verify the angle sign and units (one full turret revolution == 360 deg) and the zero.
3. Fill in turret values, set `HARDWARE_CONFIGURED=true`. Turret clear of obstacles, low `MAX_CONTROL_VOLTS` (e.g. 3 V), a person on
   the power switch. Command small targets; verify direction, limits cut power, no oscillation; then raise voltage.
4. Flywheel on SDK path: spin up, check ready/stable, `Stop` works. No balls.
5. Identify the flywheel, switch `MODEL_IDENTIFIED=true`, repeat at low speed first; verify power in range, battery sag behaviour, fault -> 0.
6. Stopper/feed with no flywheel speed: timings, jam behaviour. Then one ball at low velocity, then 3-ball sequence.
7. Enable `REQUIRE_LOCALIZATION`, zone and `REQUIRE_TURRET_ALIGNED`; shoot while stationary, then slowly moving.
8. Moving shots on a taped course; confirm the window timing; finally the real autonomous; verify auto -> teleop hand-over.
