package org.firstinspires.ftc.teamcode.calibration;

/**
 * Hard caps used by the HARDWARE calibration procedures. Deliberately compile-time constants (not tunable from a
 * file): calibration is the riskiest time to run mechanisms, so these only ever change by editing source.
 */
public final class CalConfig {
    private CalConfig() {}

    /** Turret voltage cap while calibrating (V). Production cap may be higher. */
    public static final double TURRET_MAX_VOLTS = 3.0;
    /** Open-loop flywheel power cap for sysid / scaling (fraction). */
    public static final double FLYWHEEL_MAX_POWER = 0.7;
    /** Intake/feed power cap while calibrating. */
    public static final double INTAKE_MAX_POWER = 0.6;
    /** Refuse hardware procedures below this battery voltage (V). */
    public static final double MIN_BATTERY_V = 10.5;
    /** Servo slew used when jogging the stopper by hand (position units per second). */
    public static final double STOPPER_JOG_RATE = 0.5;
    /** Inset applied to measured turret hard stops when proposing limits (rad). */
    public static final double LIMIT_INSET_RAD = Math.toRadians(5);
    /** A procedure that runs longer than this without finishing is aborted (s). */
    public static final double MAX_PROCEDURE_SECONDS = 120;
}
