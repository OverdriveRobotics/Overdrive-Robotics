package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.math.Pose;

/**
 * Shared state between commands and between autonomous and teleop.
 * Everything here is static so it survives OpMode (and RobotHardware) re-creation.
 * Hardware state is NOT trusted across OpModes: RobotHardware.init() calls
 * {@link #invalidateHardwareState()} because the SDK stops all motors between OpModes.
 */
public class Storage {
    public static Pose autoEndPose = new Pose(0, 0, 0);

    public boolean ready = false;

    public enum IntakeState { IDLE, INTAKING, REVERSING, FEEDING, JAMMED }

    /** Commanded flywheel velocity in ticks/sec. 0 = flywheel not commanded. */
    public static volatile double flywheelTargetVelocity = 0;
    /** Last measured flywheel velocity (ticks/sec) and the System.nanoTime() it was taken at. */
    public static volatile double flywheelMeasuredVelocity = 0;
    public static volatile long flywheelMeasuredNanos = 0;
    /** True only while the measured velocity was last seen inside tolerance. Always re-verify before shooting. */
    public static volatile boolean flywheelReady = false;
    /** Non-null when the flywheel is in a fault/invalid state. Cleared by the next successful spin-up. */
    public static volatile String flywheelFault = null;

    public static volatile IntakeState intakeState = IntakeState.IDLE;

    public static volatile boolean shootInProgress = false;
    /** Balls successfully shot in the current/last shoot sequence (reset at the start of each shoot command). */
    public static volatile int ballsShot = 0;
    /** Total balls shot since the program was initialised (accumulates across shoot commands). */
    public static volatile int totalBallsShot = 0;
    /** Last failure reported by any robot command, or null. */
    public static volatile String lastFailure = null;

    /** Max age of a stored measurement before it is considered stale. */
    public static final long MAX_SAMPLE_AGE_NANOS = 100_000_000L;

    public static boolean isFlywheelReadyFresh() {
        return flywheelReady && flywheelFault == null && flywheelTargetVelocity > 0
                && (System.nanoTime() - flywheelMeasuredNanos) <= MAX_SAMPLE_AGE_NANOS;
    }

    public static void reportFailure(String source, String message) {
        lastFailure = source + ": " + message;
        com.qualcomm.robotcore.util.RobotLog.ee("RobotCommands", lastFailure);
    }

    /** Called at OpMode init: motors are stopped by the SDK between OpModes, so stored flywheel/intake state is stale. */
    public static void invalidateHardwareState() {
        flywheelTargetVelocity = 0;
        flywheelMeasuredVelocity = 0;
        flywheelMeasuredNanos = 0;
        flywheelReady = false;
        flywheelFault = null;
        intakeState = IntakeState.IDLE;
        shootInProgress = false;
        ballsShot = 0;
        lastFailure = null;
    }
}
