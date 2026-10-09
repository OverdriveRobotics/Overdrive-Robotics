package org.firstinspires.ftc.teamcode.control;

/**
 * Turret hardware + model parameters. EVERY value marked PLACEHOLDER is NOT hardware data: the turret motor
 * model, gear train and inertia are not recorded anywhere in this project. Fill them in (procedure in
 * docs/MOVING_SHOT_SYSTEM.md) and then set {@link #HARDWARE_CONFIGURED} to true. While it is false,
 * AimTurret computes and reports everything but never drives the turret motor.
 *
 * Conventions: angles in radians, 0 = turret pointing along robot forward, positive = counter-clockwise
 * (same sense as Pedro heading). Positive motor power must increase the encoder; if it does not, set
 * {@link #ENCODER_SIGN} = -1 (and verify the motor direction in RobotHardware) so that positive power always
 * increases the reported angle.
 */
public final class TurretConfig {
    private TurretConfig() {}

    /**
     * Safety interlock. false = turret motor is never powered by AimTurret. Deliberately a non-final static so the
     * unit tests (and a tuning dashboard) can flip it; the committed value must stay false until calibrated.
     */
    public static boolean HARDWARE_CONFIGURED = false;

    // ---- Motor model. Specs of the shaft the encoder is mounted on (i.e. including any internal gearbox). ----
    /** PLACEHOLDER: motor+internal-gearbox output-shaft free speed, rad/s (datasheet). */
    public static final double SHAFT_FREE_SPEED_RAD_S = 20.0;
    /** PLACEHOLDER: shaft stall torque, N*m (datasheet). */
    public static final double SHAFT_STALL_TORQUE_NM = 3.0;
    /** PLACEHOLDER: stall current, A (datasheet). */
    public static final double STALL_CURRENT_A = 9.0;
    /** Voltage the datasheet numbers above were measured at. */
    public static final double NOMINAL_VOLTAGE = 12.0;

    // ---- Gearing / encoder. Do NOT double count: the internal gearbox is already inside the shaft figures. ----
    /** PLACEHOLDER: encoder ticks per revolution of THE SAME SHAFT whose speed/torque are given above. */
    public static final double SHAFT_TICKS_PER_REV = 1000.0;
    /** PLACEHOLDER: EXTERNAL reduction only = shaft revolutions per turret revolution (belt/gear ratio, &gt; 0). */
    public static final double SHAFT_REVS_PER_TURRET_REV = 3.0;
    /** +1 or -1: sign so that positive motor power increases the turret angle read from the encoder. */
    public static final int ENCODER_SIGN = 1;
    /** Turret angle at OpMode init (encoder zero). Turret must be centred/known at init. */
    public static final double START_ANGLE_RAD = 0.0;

    // ---- Mechanical limits ----
    /** PLACEHOLDER: true only if the turret can spin without cable/hard-stop limits. */
    public static final boolean CONTINUOUS = false;
    /** PLACEHOLDER: reachable range, rad (ignored when CONTINUOUS). Keep inside the true hard stops. */
    public static final double MIN_ANGLE_RAD = -Math.PI / 2;
    public static final double MAX_ANGLE_RAD = Math.PI / 2;
    /** Power toward a limit is cut within this margin of it. */
    public static final double LIMIT_MARGIN_RAD = Math.toRadians(3);

    // ---- Dynamics (turret output side) ----
    /** PLACEHOLDER: turret + payload inertia about the turret axis, kg*m^2 (from CAD or a swing test). */
    public static final double LOAD_INERTIA_KG_M2 = 0.02;
    /** PLACEHOLDER: viscous friction at the turret, N*m per rad/s (identify experimentally). */
    public static final double VISCOUS_FRICTION = 0.01;
    /** PLACEHOLDER: static friction expressed in volts (smallest voltage that creeps the turret). */
    public static final double STATIC_FRICTION_VOLTS = 0.0;

    // ---- Controller ----
    /** Nominal control period the LQR gain is designed for (OpMode loops typically run 10-30 ms). */
    public static final double SAMPLE_PERIOD_S = 0.02;
    /** Bryson-rule weights: the error that should cost as much as the max input. */
    public static final double MAX_ANGLE_ERROR_RAD = Math.toRadians(5);
    public static final double MAX_VELOCITY_ERROR_RAD_S = 1.0;
    public static final double MAX_CONTROL_VOLTS = 12.0;
    /** Slew limit of the internal reference so a target switch is not a step input. */
    public static final double MAX_REFERENCE_VELOCITY_RAD_S = 6.0;
    /** First-order low-pass on the measured velocity, 0 &lt; alpha &lt;= 1 (1 = no filtering). */
    public static final double VELOCITY_FILTER_ALPHA = 0.5;
    /** Integral action (volts per rad*s) to remove persistent offset; 0 = off. Clamped by the limit below. */
    public static final double INTEGRAL_GAIN = 0.0;
    public static final double INTEGRAL_LIMIT_VOLTS = 1.5;

    // ---- Alignment ----
    public static final double ALIGN_TOLERANCE_RAD = Math.toRadians(2);
    public static final double ALIGN_VELOCITY_TOL_RAD_S = 0.5;
    /** Pose/velocity older than this is not trusted for aiming or readiness. */
    public static final double MAX_POSE_AGE_MS = 100;
    /** ASSUMPTION: Follower.velocity() is expressed in the FIELD frame. Set false if it is robot-frame. */
    public static final boolean ROBOT_VELOCITY_IS_FIELD_FRAME = true;
}
