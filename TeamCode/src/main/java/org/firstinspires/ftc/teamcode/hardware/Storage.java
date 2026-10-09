package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.math.Pose;

/**
 * Shared state between commands and between autonomous and teleop.
 * Everything here is static so it survives OpMode (and RobotHardware) re-creation.
 * Hardware state is NOT trusted across OpModes: RobotHardware.init() calls
 * {@link #invalidateHardwareState()} because the SDK stops all motors between OpModes.
 * Field target points and the active target index are CONFIGURATION and deliberately survive it.
 */
public class Storage {
    public static Pose autoEndPose = new Pose(0, 0, 0);

    public boolean ready = false;

    public enum IntakeState { IDLE, INTAKING, REVERSING, FEEDING, JAMMED }

    // ------------------------------------------------------------------ field targets (runtime switchable)

    /** Immutable field point (same units as the Pedro pose, inches). NaN coordinates mean "not set". */
    public static final class FieldPoint {
        public final double x, y;
        public FieldPoint(double x, double y) { this.x = x; this.y = y; }
        public boolean isValid() { return Double.isFinite(x) && Double.isFinite(y); }
        @Override public String toString() { return "(" + x + ", " + y + ")"; }
    }

    public static final int TARGET_COUNT = 2;
    /** Target points A (index 0) and B (index 1). Unset by default: set them from field measurements. */
    public static volatile FieldPoint targetA = new FieldPoint(Double.NaN, Double.NaN);
    public static volatile FieldPoint targetB = new FieldPoint(Double.NaN, Double.NaN);
    public static volatile int activeTargetIndex = 0;

    /** Replaces target {@code index}. Rejects an out-of-range index or non-finite coordinates (returns false). */
    public static boolean setTarget(int index, double x, double y) {
        if (index < 0 || index >= TARGET_COUNT || !Double.isFinite(x) || !Double.isFinite(y)) return false;
        if (index == 0) targetA = new FieldPoint(x, y); else targetB = new FieldPoint(x, y);
        return true;
    }

    /** Selects the active target. Invalid indices are rejected and leave the selection unchanged. */
    public static boolean selectTarget(int index) {
        if (index < 0 || index >= TARGET_COUNT) return false;
        activeTargetIndex = index;
        return true;
    }

    public static void toggleTarget() { activeTargetIndex = (activeTargetIndex + 1) % TARGET_COUNT; }

    /** The active target, or null if the index is somehow invalid. May hold NaN coordinates: check isValid(). */
    public static FieldPoint activeTarget() {
        int i = activeTargetIndex;
        return i == 0 ? targetA : i == 1 ? targetB : null;
    }

    // ------------------------------------------------------------------ localization

    public static volatile long poseUpdateNanos = 0;

    // ------------------------------------------------------------------ turret status (written by AimTurret)

    /** Encoder ticks captured as the turret zero at init. */
    public static volatile int turretZeroTicks = 0;
    public static volatile double turretAngle = Double.NaN;
    public static volatile double turretVelocity = Double.NaN;
    public static volatile double turretDesiredAngle = Double.NaN;
    public static volatile double turretError = Double.NaN;
    public static volatile double turretPower = 0;
    public static volatile boolean turretSaturated = false;
    public static volatile boolean turretReachable = false;
    public static volatile long turretUpdateNanos = 0;
    /** Non-null while the turret loop is inhibited or faulted (including "not calibrated"). */
    public static volatile String turretFault = null;

    // ------------------------------------------------------------------ flywheel (written by FlywheelRegulator)

    /** Commanded flywheel velocity in ticks/sec. 0 = flywheel not commanded. */
    public static volatile double flywheelTargetVelocity = 0;
    /** Last measured flywheel velocity (ticks/sec) and the RobotClock time it was taken at. */
    public static volatile double flywheelMeasuredVelocity = 0;
    public static volatile long flywheelMeasuredNanos = 0;
    /** True only while the measured velocity has been inside tolerance for the full stability period. */
    public static volatile boolean flywheelReady = false;
    /** Non-null when the flywheel is in a fault/invalid state. Cleared by the next successful spin-up request. */
    public static volatile String flywheelFault = null;
    public static volatile double flywheelTolerance = 50;
    public static volatile double flywheelStableMs = 150;
    /** Time (ms) the velocity has been continuously in tolerance; 0 when out of tolerance. */
    public static volatile double flywheelStableForMs = 0;
    public static volatile double flywheelPower = 0;
    public static volatile String flywheelMode = "off";

    public static volatile IntakeState intakeState = IntakeState.IDLE;

    // ------------------------------------------------------------------ shooting

    public static volatile boolean shootInProgress = false;
    public static volatile String shootPhase = "IDLE";
    /** Balls successfully shot in the current/last shoot sequence (reset at the start of each shoot command). */
    public static volatile int ballsShot = 0;
    /** Total balls shot since the program was initialised (accumulates across shoot commands). */
    public static volatile int totalBallsShot = 0;
    /** Last failure reported by any robot command, or null. */
    public static volatile String lastFailure = null;
    /** Latest result of the live readiness evaluation (see ShootReadiness). */
    public static volatile boolean readyToShoot = false;
    public static volatile String notReadyReason = "not evaluated";

    /** Max age of a stored measurement before it is considered stale. */
    public static final long MAX_SAMPLE_AGE_NANOS = 100_000_000L;

    public static boolean isFlywheelReadyFresh() {
        return flywheelReady && flywheelFault == null && flywheelTargetVelocity > 0
                && (RobotClock.nanos() - flywheelMeasuredNanos) <= MAX_SAMPLE_AGE_NANOS;
    }

    public static void reportFailure(String source, String message) {
        lastFailure = source + ": " + message;
        try {
            com.qualcomm.robotcore.util.RobotLog.ee("RobotCommands", lastFailure);
        } catch (Throwable ignored) {
            // logging must never break a safety path (also keeps JVM unit tests free of Android stubs)
        }
    }

    /** Called at OpMode init: motors are stopped by the SDK between OpModes, so stored hardware state is stale. */
    public static void invalidateHardwareState() {
        flywheelTargetVelocity = 0;
        flywheelMeasuredVelocity = 0;
        flywheelMeasuredNanos = 0;
        flywheelReady = false;
        flywheelFault = null;
        flywheelStableForMs = 0;
        flywheelPower = 0;
        flywheelMode = "off";
        intakeState = IntakeState.IDLE;
        shootInProgress = false;
        shootPhase = "IDLE";
        ballsShot = 0;
        lastFailure = null;
        readyToShoot = false;
        notReadyReason = "not evaluated";
        turretAngle = turretVelocity = turretDesiredAngle = turretError = Double.NaN;
        turretPower = 0;
        turretSaturated = false;
        turretReachable = false;
        turretUpdateNanos = 0;
        turretFault = null;
        poseUpdateNanos = 0;
    }
}
