package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

import java.util.function.BooleanSupplier;

import static com.pedropathing.ivy.commands.Commands.infinite;

/**
 * Factories for reusable robot commands. Each call returns a fresh, independent command.
 *
 * Resource ownership (Ivy requirements):
 *  - spinUpFlywheel / stopFlywheel : flywheel (shooterLeft+shooterRight, keyed on shooterRight)
 *  - shoot                         : stopper servo + intakeAndTransferMotor (reads the flywheel, never commands it)
 *  - runIntake                     : intakeAndTransferMotor
 *  - drive (Pedro follow/hold)     : none of the above, so it runs in parallel with all of these.
 *
 * Example: parallel(follow(follower, toShootSpot), spinUpFlywheel(robot, 1800)).then(shoot(robot, 3))
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
        return new ShootBalls(robot, balls, shootVelocity, RobotConfig.FLYWHEEL_TOLERANCE);
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
}
