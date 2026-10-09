package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.hardware.RobotClock;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

/** Flywheel measurement and target-request helpers shared by the commands. */
final class FlywheelUtil {
    private FlywheelUtil() {}

    /** Resource key for the flywheel: every command that writes the shooter motors must require this. */
    static Object flywheelKey(RobotHardware robot) { return Resources.flywheel(robot); }

    static boolean hardwareAvailable(RobotHardware robot) {
        return robot != null && robot.shooterLeft != null && robot.shooterRight != null;
    }

    /**
     * Reads both shooter motors, publishes the average to Storage and returns it, or NaN if invalid
     * (a non-finite reading from either side).
     */
    static double sample(RobotHardware robot) {
        double l = robot.shooterLeft.getVelocity();
        double r = robot.shooterRight.getVelocity();
        boolean ok = Double.isFinite(l) && Double.isFinite(r);
        double v = ok ? (l + r) / 2.0 : Double.NaN;
        Storage.flywheelMeasuredNanos = RobotClock.nanos();
        if (ok) {
            Storage.flywheelMeasuredVelocity = v;
        } else {
            Storage.flywheelReady = false;
        }
        return v;
    }

    static boolean inTolerance(double v, double target, double tol) {
        return Double.isFinite(v) && target > 0 && Math.abs(v - target) <= tol;
    }

    /**
     * Records the requested flywheel target. Returns true if this is a NEW request (different target, or a fault
     * is being cleared); an identical repeated request changes nothing and does not reset readiness.
     */
    static boolean requestTarget(double target, double tolerance, double stableMs) {
        boolean changed = Math.abs(Storage.flywheelTargetVelocity - target) > 1e-6 || Storage.flywheelFault != null;
        if (changed) {
            Storage.flywheelReady = false;
            Storage.flywheelStableForMs = 0;
            Storage.flywheelFault = null;
            Storage.flywheelTargetVelocity = target;
        }
        Storage.flywheelTolerance = tolerance;
        Storage.flywheelStableMs = stableMs;
        return changed;
    }

    private static FlywheelRegulator active;

    /** Makes sure exactly one regulator (the only writer of the shooter motors) is scheduled. */
    static void ensureRegulator(RobotHardware robot) {
        FlywheelRegulator r = active;
        if (r != null && Scheduler.isScheduled(r)) return;
        active = new FlywheelRegulator(robot);
        Scheduler.schedule(active);
    }
}
