package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

import java.util.function.BooleanSupplier;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Factories for reusable robot commands. Each call returns a fresh, independent command.
 *
 * Resource ownership (Ivy requirements):
 *  - flywheelRegulator (internal)  : flywheel (shooterLeft+shooterRight, keyed on shooterRight) - the ONLY motor writer
 *  - stopFlywheel                  : flywheel (priority 10, interrupts the regulator)
 *  - spinUpFlywheel                : none (sets the regulator's target and waits for measured readiness)
 *  - aimTurret                     : turret motor
 *  - shoot                         : stopper servo + intakeAndTransferMotor (reads the flywheel, never commands it)
 *  - runIntake                     : intakeAndTransferMotor
 *  - drive (Pedro follow/hold)     : none of the above, so it runs in parallel with all of these.
 *
 * Moving shot: see {@link #movingShot}. Background loops (flywheel regulator, aimTurret, readiness monitor) are
 * scheduled once per OpMode by {@link #startShooterSystems}.
 * In teleop/auto also schedule {@link #flywheelMonitor} so Storage readiness stays fresh between commands.
 */
public final class RobotCommands {
    private RobotCommands() {}

    public static Command spinUpFlywheel(RobotHardware robot, double targetVelocity) {
        return new SpinUpFlywheel(robot, targetVelocity, RobotConfig.FLYWHEEL_TOLERANCE,
                RobotConfig.FLYWHEEL_STABLE_MS, RobotConfig.FLYWHEEL_SPINUP_TIMEOUT_MS);
    }

    public static Command spinUpFlywheel(RobotHardware robot, double targetVelocity, double tolerance,
                                         double stableMs, double timeoutMs) {
        return new SpinUpFlywheel(robot, targetVelocity, tolerance, stableMs, timeoutMs);
    }

    /** Explicit flywheel stop (the only command that turns it off). */
    public static Command stopFlywheel(RobotHardware robot) { return new StopFlywheel(robot); }

    /** Shoots {@code balls} at whatever velocity the flywheel is currently commanded to. */
    public static Command shoot(RobotHardware robot, int balls) {
        return new ShootBalls(robot, balls, Double.NaN, RobotConfig.FLYWHEEL_TOLERANCE);
    }

    /** Shoots {@code balls}; requests a spin-up to {@code shootVelocity} first if not already commanded. */
    public static Command shoot(RobotHardware robot, int balls, double shootVelocity) {
        // NaN is ShootBalls' internal "use the current target" marker; an explicit NaN here is a caller error.
        return new ShootBalls(robot, balls, Double.isNaN(shootVelocity) ? Double.NEGATIVE_INFINITY : shootVelocity,
                RobotConfig.FLYWHEEL_TOLERANCE);
    }

    /** Runs the intake in IN or OUT until cancelled. */
    public static Command runIntake(RobotHardware robot, boolean reverse) {
        return runIntake(robot, reverse, RobotConfig.INTAKE_POWER, 0, null);
    }

    /** durationMs &lt;= 0 means no time limit; stopWhen may be null. */
    public static Command runIntake(RobotHardware robot, boolean reverse, double power, double durationMs,
                                    BooleanSupplier stopWhen) {
        return new RunIntake(robot, reverse ? Storage.IntakeState.REVERSING : Storage.IntakeState.INTAKING,
                power, durationMs, stopWhen);
    }

    /** Requirement-free background command keeping Storage's measured velocity/readiness fresh. */
    public static Command flywheelMonitor(RobotHardware robot, double tolerance) {
        return infinite(() -> {
            if (!FlywheelUtil.hardwareAvailable(robot) || Storage.flywheelTargetVelocity <= 0) return;
            double v = FlywheelUtil.sample(robot);
            // Only ever downgrades readiness here; promotion to ready requires a stable window (spin-up/shoot).
            if (!FlywheelUtil.inTolerance(v, Storage.flywheelTargetVelocity, tolerance)) Storage.flywheelReady = false;
        });
    }

    /** Continuous turret aiming at Storage's active target; runs until cancelled. */
    public static Command aimTurret(RobotHardware robot) { return new AimTurret(robot); }

    /** Schedules the (single) flywheel regulator if it is not already running. Safe to call repeatedly. */
    public static void ensureFlywheelRegulator(RobotHardware robot) { FlywheelUtil.ensureRegulator(robot); }

    /** Requirement-free background command that re-evaluates {@link ShootStatus} into Storage every loop. */
    public static Command readinessMonitor(RobotHardware robot) {
        return infinite(() -> {
            org.firstinspires.ftc.teamcode.control.ShootingReadiness.Result r =
                    ShootStatus.evaluate(robot, !Storage.shootInProgress, "shot in progress");
            Storage.readyToShoot = r.ready;
            Storage.notReadyReason = r.reason;
        });
    }

    /**
     * Schedules the background loops a moving shot needs: flywheel regulator, turret aiming, readiness monitor.
     * Call once after {@code Scheduler.reset()} in the OpMode's start(). They own distinct resources, so they
     * run alongside path following and shooting.
     */
    public static void startShooterSystems(RobotHardware robot) {
        ensureFlywheelRegulator(robot);
        Scheduler.schedule(aimTurret(robot), readinessMonitor(robot));
    }

    /** Shoots with a custom window timeout (how long to wait for readiness before each ball). */
    public static Command shoot(RobotHardware robot, int balls, double shootVelocity, double readyTimeoutMs) {
        return new ShootBalls(robot, balls, Double.isNaN(shootVelocity) ? Double.NEGATIVE_INFINITY : shootVelocity,
                RobotConfig.FLYWHEEL_TOLERANCE, readyTimeoutMs);
    }

    /**
     * Follows {@code path} while spinning up and shooting: the path is NOT awaited before shooting starts.
     * ShootBalls requests the flywheel velocity itself and waits (up to {@code windowTimeoutMs} per ball) for the
     * live readiness window; it finishes when all balls are shot or it times out. The group ends when BOTH the path
     * and the shooting have ended. Needs {@link #startShooterSystems} running for turret aiming and readiness.
     */
    public static Command movingShot(RobotHardware robot, Path path, int balls, double shootVelocity,
                                     double windowTimeoutMs) {
        return parallel(
                follow(robot.follower, path),
                shoot(robot, balls, shootVelocity, windowTimeoutMs));
    }
}
