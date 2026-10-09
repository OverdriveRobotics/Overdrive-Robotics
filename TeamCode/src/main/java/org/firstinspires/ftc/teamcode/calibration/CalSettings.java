package org.firstinspires.ftc.teamcode.calibration;

/**
 * Operator-adjustable settings of the calibration procedures themselves (durations, step sizes, test path). Registered
 * in the ParamRegistry under "cal.*", so they are edited and saved exactly like robot parameters - no source edits.
 * Path poses are deliberately UNSET (NaN): they are field measurements, and the moving-shot stages refuse to drive until
 * you provide them.
 */
public final class CalSettings {
    private CalSettings() {}

    public static double FLYWHEEL_STEP_HOLD_S = 6;
    public static double DISTURBANCE_MS = 200;
    public static double TURRET_STEP_DEG = 20;
    public static double TURRET_KNOWN_ANGLE_DEG = 90;
    public static double TURRET_AIM_SECONDS = 15;
    public static double MONITOR_SECONDS = 20;
    public static int STAGE_BALLS = 3;
    public static double WINDOW_MS = 4000;
    public static double STAGE_SECONDS = 30;
    public static double PATH_START_X = Double.NaN, PATH_START_Y = Double.NaN, PATH_START_HEADING_DEG = Double.NaN;
    public static double PATH_END_X = Double.NaN, PATH_END_Y = Double.NaN, PATH_END_HEADING_DEG = Double.NaN;

    /** Start pose of the test path, or null if not configured. */
    public static com.pedropathing.math.Pose start() {
        return finite(PATH_START_X, PATH_START_Y, PATH_START_HEADING_DEG) ? new com.pedropathing.math.Pose(PATH_START_X, PATH_START_Y, Math.toRadians(PATH_START_HEADING_DEG)) : null;
    }
    public static com.pedropathing.math.Pose end() {
        return finite(PATH_END_X, PATH_END_Y, PATH_END_HEADING_DEG) ? new com.pedropathing.math.Pose(PATH_END_X, PATH_END_Y, Math.toRadians(PATH_END_HEADING_DEG)) : null;
    }
    private static boolean finite(double... v) { for (double d : v) if (!Double.isFinite(d)) return false; return true; }
}
