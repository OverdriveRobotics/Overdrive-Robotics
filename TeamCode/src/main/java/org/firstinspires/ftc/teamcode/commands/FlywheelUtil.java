package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

/** Flywheel measurement helpers shared by the commands. */
final class FlywheelUtil {
    private FlywheelUtil() {}

    /** Resource key for the flywheel: every command that writes the shooter motors must require this. */
    static Object flywheelKey(RobotHardware robot) { return robot.shooterRight; }

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
        Storage.flywheelMeasuredNanos = System.nanoTime();
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
}
