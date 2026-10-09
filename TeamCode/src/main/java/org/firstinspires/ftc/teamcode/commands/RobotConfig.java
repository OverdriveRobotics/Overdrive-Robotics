package org.firstinspires.ftc.teamcode.commands;

/** Tunable constants for the robot commands. All values marked ASSUMPTION need on-robot confirmation. */
public final class RobotConfig {
    private RobotConfig() {}

    /** ASSUMPTION: flywheel velocity units are ticks/sec (DcMotorEx.setVelocity). Upper sanity bound (28 tpr * 6000 rpm / 60). */
    public static final double MAX_FLYWHEEL_VELOCITY = 2800;
    public static final double FLYWHEEL_TOLERANCE = 50;
    public static final double FLYWHEEL_STABLE_MS = 150;
    public static final double FLYWHEEL_SPINUP_TIMEOUT_MS = 5000;

    /** ASSUMPTION: servo "stoper" positions; closed blocks balls, open lets them through. */
    public static final double STOPPER_CLOSED = 0.0;
    public static final double STOPPER_OPEN = 0.5;
    /** ASSUMPTION (no servo feedback): time for the stopper to finish opening / closing. Calibrate by video. */
    public static final double STOPPER_SETTLE_MS = 150;
    public static final double STOPPER_CLOSE_SETTLE_MS = 150;

    public static final int MAX_BALLS = 3;
    /** Feed motor power while pushing a ball into the flywheel. */
    public static final double FEED_POWER = 1.0;
    public static final double READY_TIMEOUT_MS = 4000;
    public static final double SHOT_TIMEOUT_MS = 1500;
    public static final double RECOVER_TIMEOUT_MS = 3000;
    /**
     * No ball sensor exists, so a shot is detected as a flywheel velocity drop of at least this many
     * ticks/sec (below the peak seen while feeding), for DROP_CONFIRM_LOOPS consecutive loops.
     */
    public static final double SHOT_DROP_THRESHOLD = 150;
    public static final int DROP_CONFIRM_LOOPS = 2;
    /** Optional feed-motor encoder check (ASSUMPTION: intake motor encoder is not necessarily plugged in). */
    public static final boolean CHECK_FEED_ENCODER = false;
    public static final double FEED_MIN_VELOCITY = 50;
    public static final double FEED_CHECK_AFTER_MS = 400;

    public static final double INTAKE_POWER = 1.0;
    /** Jam detection by motor current. ASSUMPTION thresholds; disabled by default. */
    public static final boolean DETECT_INTAKE_JAM = false;
    public static final double INTAKE_JAM_AMPS = 6.0;
    public static final double INTAKE_JAM_MS = 400;

    // ---- Shooting-readiness gates (non-final statics so tests / a tuning dashboard can change them). Defaults keep the pre-turret behaviour; enable as each subsystem is calibrated. ----
    /** Require a fresh, finite Pedro pose before every shot. */
    public static boolean REQUIRE_LOCALIZATION = false;
    /** Require the turret to be aligned (needs TurretConfig.HARDWARE_CONFIGURED and valid target points). */
    public static boolean REQUIRE_TURRET_ALIGNED = false;
    /** Require the robot to be inside the rectangle below (field inches). PLACEHOLDER bounds: measure the real zone. */
    public static boolean SHOOT_ZONE_ENABLED = false;
    public static double SHOOT_ZONE_MIN_X = Double.NaN, SHOOT_ZONE_MAX_X = Double.NaN;
    public static double SHOOT_ZONE_MIN_Y = Double.NaN, SHOOT_ZONE_MAX_Y = Double.NaN;
    /** Max robot translational speed (inches/s) for a shot; PLACEHOLDER until shot-on-the-move is characterised. */
    public static final double MAX_SHOOT_SPEED = 30;
    /** ASSUMPTION: flywheel velocity (ticks/s) used for target A / B by the OpMode helpers. */
    public static final double FLYWHEEL_VELOCITY_A = 1800;
    public static final double FLYWHEEL_VELOCITY_B = 1800;
}
