# Subsystem inventory (as inspected)

Facts come from the code in this repository, the installed library bytecode (Pedro 3.0.1, Ivy 1.1.1, FTC SDK 12.0.0) and the tests.
"Assumption" = not verifiable without the robot.

| Subsystem | Hardware / class | Dependencies | Feedback available | Tunable parameters (registry keys) | Safety limits | Outstanding assumptions |
|---|---|---|---|---|---|---|
| Hardware map | `RobotHardware` (`shooterRight/Left`, `turret`, `intakeAndTransferMotor` DcMotorEx; `stoper`, `hood` Servo; Pedro follower; first voltage sensor) | FTC SDK, Pedro `Constants` | battery voltage | - | `safeShutdown()` defined safe state | motor types/gearing unknown; `hood` unused |
| Flywheel | `FlywheelRegulator`, `FlywheelLqrController`, `FlywheelUtil` | battery voltage, both encoders | velocity (ticks/s) per motor, averaged | `flywheel.*`, `robot.FLYWHEEL_*` | `flywheel.MAX_CONTROL_VOLTS`, `MAX_PLAUSIBLE_VELOCITY`, `robot.MAX_FLYWHEEL_VELOCITY` | both motors drive one wheel; SDK PIDF values (all 1,1,1,1) are placeholders; LQR advantage unmeasured |
| Turret | `AimTurret`, `TurretStateSpaceController`, `TurretUtil` | Pedro pose/velocity, battery, encoder | **relative** encoder only (zero at init), velocity | `turret.*`, `target.*` | `turret.MAX_CONTROL_VOLTS`, limits, `LIMIT_MARGIN_RAD`; disabled until `HARDWARE_CONFIGURED` | motor/gearing/inertia unknown; zero assumes centred at init |
| Intake / transfer | `RunIntake` | feed motor shared with shooting | optional motor current/velocity | `robot.INTAKE_POWER`, `FEED_POWER`, jam settings | cleanup in `end()` | no ball sensor; direction is a wiring fact |
| Stopper | servo `stoper` in `ShootBalls` | - | **none** | `robot.STOPPER_*` | closed on every exit | settle times are observations |
| Shooting | `ShootBalls`, `ShootStatus`, `ShootingReadiness` | flywheel, turret, pose, stopper, feed | live Storage state | `robot.*` thresholds, zone, `MAX_SHOOT_SPEED` | readiness gates, timeouts | shot = RPM drop (no ball sensor) |
| Localization | Pedro `Follower` (Pinpoint) | `pedro/Constants` | pose, field-frame velocity (software-verified) | `turret.ROBOT_VELOCITY_IS_FIELD_FRAME` | stale pose cuts output | Foresight follower gains untuned; raw Pinpoint velocity frame to confirm |
| Scheduler | Ivy `Scheduler`, `Resources`, `RobotCommands`, `AutoRoutines` | - | - | - | `Scheduler.reset()` + `safeShutdown` | none (behaviour tested) |
| State | `Storage`, `RobotClock` | - | - | `target.*` | `invalidateHardwareState()` per OpMode | - |
| Config | `tuning/*` | file system | - | all of the above | validator | - |
