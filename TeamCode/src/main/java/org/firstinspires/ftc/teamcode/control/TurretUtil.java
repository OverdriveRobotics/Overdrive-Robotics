package org.firstinspires.ftc.teamcode.control;

/** Turret geometry and unit conversions. Pure functions of their arguments: unit-testable without hardware. */
public final class TurretUtil {
    private TurretUtil() {}

    // ---- Unit conversion (explicit-parameter forms for tests; wrappers use TurretConfig) ----

    /** Encoder ticks (relative to the zero capture) to turret angle. */
    public static double ticksToAngle(double ticksFromZero, double shaftTicksPerRev, double shaftRevsPerTurretRev,
                                      int sign, double startAngle) {
        return startAngle + sign * ticksFromZero / shaftTicksPerRev / shaftRevsPerTurretRev * 2 * Math.PI;
    }

    public static double angleToTicks(double angle, double shaftTicksPerRev, double shaftRevsPerTurretRev,
                                      int sign, double startAngle) {
        return sign * (angle - startAngle) / (2 * Math.PI) * shaftRevsPerTurretRev * shaftTicksPerRev;
    }

    public static double ticksPerSecToRadPerSec(double tps, double shaftTicksPerRev, double shaftRevsPerTurretRev,
                                                int sign) {
        return sign * tps / shaftTicksPerRev / shaftRevsPerTurretRev * 2 * Math.PI;
    }

    public static double ticksToAngle(double ticksFromZero) {
        return ticksToAngle(ticksFromZero, TurretConfig.SHAFT_TICKS_PER_REV, TurretConfig.SHAFT_REVS_PER_TURRET_REV,
                TurretConfig.ENCODER_SIGN, TurretConfig.START_ANGLE_RAD);
    }

    public static double ticksPerSecToRadPerSec(double tps) {
        return ticksPerSecToRadPerSec(tps, TurretConfig.SHAFT_TICKS_PER_REV, TurretConfig.SHAFT_REVS_PER_TURRET_REV,
                TurretConfig.ENCODER_SIGN);
    }

    // ---- Geometry ----

    /** Field-frame bearing from the robot to the target, atan2(dy, dx). NaN if any input is non-finite. */
    public static double fieldAzimuth(double robotX, double robotY, double targetX, double targetY) {
        return Math.atan2(targetY - robotY, targetX - robotX);
    }

    /** Desired turret angle relative to the robot, wrapped to (-PI, PI]. */
    public static double desiredRelativeAngle(double robotX, double robotY, double robotHeading,
                                              double targetX, double targetY) {
        return AngleUtil.wrap(fieldAzimuth(robotX, robotY, targetX, targetY) - robotHeading);
    }

    /**
     * Rate of change of the desired relative turret angle caused by robot motion (rad/s). With the target fixed in
     * the field and the robot moving at field velocity (vx, vy) and turning at omega:
     * d(bearing)/dt = (dy*vx - dx*vy) / (dx^2 + dy^2), d(relative)/dt = that - omega. Returns 0 right on top of
     * the target (bearing undefined).
     */
    public static double desiredRelativeRate(double robotX, double robotY, double fieldVx, double fieldVy,
                                             double omega, double targetX, double targetY) {
        double dx = targetX - robotX, dy = targetY - robotY;
        double r2 = dx * dx + dy * dy;
        if (!(r2 > 1e-6)) return 0;
        return (dy * fieldVx - dx * fieldVy) / r2 - omega;
    }

    /** Rotates a robot-frame velocity into the field frame. */
    public static double[] robotToFieldVelocity(double vxRobot, double vyRobot, double heading) {
        double c = Math.cos(heading), s = Math.sin(heading);
        return new double[]{vxRobot * c - vyRobot * s, vxRobot * s + vyRobot * c};
    }

    /**
     * Chooses the reachable equivalent of {@code desiredRelative} nearest to the measured angle. Returns NaN if the
     * mechanical range contains no equivalent angle (caller must then treat the target as unreachable).
     */
    public static double reachableAngle(double desiredRelative, double measuredAngle) {
        if (TurretConfig.CONTINUOUS) {
            return AngleUtil.equivalentInRange(desiredRelative, measuredAngle,
                    Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        return AngleUtil.equivalentInRange(desiredRelative, measuredAngle,
                TurretConfig.MIN_ANGLE_RAD, TurretConfig.MAX_ANGLE_RAD);
    }

    public static boolean isAligned(double errorRad, double velocityRadS, double tolRad, double velTolRadS) {
        return Double.isFinite(errorRad) && Double.isFinite(velocityRadS)
                && Math.abs(errorRad) <= tolRad && Math.abs(velocityRadS) <= velTolRadS;
    }

    public static boolean withinLimits(double angle, double min, double max) {
        return Double.isFinite(angle) && angle >= min && angle <= max;
    }

    /**
     * Cuts motor power that would drive further into a mechanical limit. Returns the (possibly zeroed) power.
     */
    public static double guardLimits(double power, double angle, double min, double max, double margin) {
        if (!Double.isFinite(angle)) return 0;
        if (angle >= max - margin && power > 0) return 0;
        if (angle <= min + margin && power < 0) return 0;
        return power;
    }
}
