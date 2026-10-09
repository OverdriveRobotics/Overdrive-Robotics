package org.firstinspires.ftc.teamcode.control;

/**
 * Flywheel model parameters. Values marked PLACEHOLDER are NOT hardware data (the flywheel motor type, inertia
 * and friction are not recorded in this project). Until {@link #MODEL_IDENTIFIED} is true the flywheel regulator
 * uses the SDK's built-in velocity PIDF (the behaviour the project had before) and the LQR path is not used.
 *
 * Most fields are non-final statics so an ACCEPTED calibration file can override them (tuning/ParamRegistry).
 *
 * Units: the controlled state is the motor-shaft velocity in encoder ticks/s, i.e. exactly what
 * DcMotorEx.getVelocity() reports and what the rest of the project (RobotConfig.MAX_FLYWHEEL_VELOCITY, targets) uses.
 */
public final class FlywheelConfig {
    private FlywheelConfig() {}

    /** Safety interlock: false = use SDK velocity PIDF, true = use LQR + voltage compensation. */
    public static boolean MODEL_IDENTIFIED = false;   // non-final only so tests/tuning can flip it; commit false until identified

    /**
     * How the plant is specified.
     * PHYSICAL: from the motor constants and inertia below.  SYSID: from measured kS/kV/kA (preferred, see doc).
     */
    public static boolean USE_SYSID_MODEL = false;

    // ---- PHYSICAL model inputs (PLACEHOLDER) ----
    /** Number of identical motors driving the flywheel (assumption: both shooter motors drive one flywheel). */
    public static int MOTOR_COUNT = 2;
    public static double SHAFT_FREE_SPEED_RAD_S = 628.0;   // PLACEHOLDER (~6000 rpm)
    public static double SHAFT_STALL_TORQUE_NM = 0.15;     // PLACEHOLDER
    public static double STALL_CURRENT_A = 9.0;            // PLACEHOLDER
    public static final double NOMINAL_VOLTAGE = 12.0;
    /** PLACEHOLDER: encoder ticks per rev of the motor shaft (matches RobotConfig's 28 tpr assumption). */
    public static double SHAFT_TICKS_PER_REV = 28.0;
    /** PLACEHOLDER: motor-shaft revolutions per flywheel revolution (1.0 = direct drive). */
    public static double SHAFT_REVS_PER_FLYWHEEL_REV = 1.0;
    /** PLACEHOLDER: flywheel inertia about its axis, kg*m^2 (CAD, or spin-down test). */
    public static double FLYWHEEL_INERTIA_KG_M2 = 3.0e-4;
    /** PLACEHOLDER: viscous drag at the flywheel, N*m per rad/s. */
    public static double VISCOUS_FRICTION = 1.0e-5;

    // ---- SYSID model inputs (PLACEHOLDER until measured). Units: volts, volts per (tick/s), volts per (tick/s^2). ----
    public static double KS_VOLTS = 0.0;
    public static double KV_VOLTS_PER_TICK_S = 12.0 / 2800.0;
    public static double KA_VOLTS_PER_TICK_S2 = 0.002;

    // ---- Controller ----
    public static final double SAMPLE_PERIOD_S = 0.02;
    /** Bryson-rule weights: a velocity error (ticks/s) that costs as much as {@link #MAX_CONTROL_VOLTS}. */
    public static double MAX_VELOCITY_ERROR = 100.0;
    public static double MAX_CONTROL_VOLTS = 12.0;
    /** Kalman std-devs (ticks/s): model uncertainty per step and encoder noise. Larger model = faster reaction. */
    public static double MODEL_STD = 50.0;
    public static double MEASUREMENT_STD = 40.0;
    /** Largest model-vs-measurement jump (ticks/s) the observer will believe in a step before treating it as a fault. */
    public static double MAX_PLAUSIBLE_VELOCITY = 1.5 * 2800;
}
