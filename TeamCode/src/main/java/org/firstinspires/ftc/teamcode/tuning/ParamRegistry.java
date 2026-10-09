package org.firstinspires.ftc.teamcode.tuning;

import org.firstinspires.ftc.teamcode.calibration.CalSettings;
import org.firstinspires.ftc.teamcode.commands.RobotConfig;
import org.firstinspires.ftc.teamcode.control.FlywheelConfig;
import org.firstinspires.ftc.teamcode.control.TurretConfig;
import org.firstinspires.ftc.teamcode.hardware.Storage;
import org.firstinspires.ftc.teamcode.tuning.Param.Category;
import org.firstinspires.ftc.teamcode.tuning.Param.Tighten;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.Supplier;

/**
 * The single list of tunable parameters. Keys are "turret.X", "flywheel.X", "robot.X" (the config class and field
 * name) plus "target.*" for the two field target points. Defaults are captured when this class loads, i.e. they are
 * the compiled values; a loaded configuration only OVERRIDES them.
 */
public final class ParamRegistry {
    private ParamRegistry() {}

    private static final Map<String, Param> PARAMS = new LinkedHashMap<>();
    private static final Map<String, Double> DEFAULTS = new LinkedHashMap<>();

    private static final Category FIX = Category.FIXED_DERIVED, HW = Category.HARDWARE_CAL, CTRL = Category.CONTROLLER,
            BEH = Category.BEHAVIOR, SAFE = Category.SAFETY, LOCK = Category.INTERLOCK;

    private static void add(String prefix, Class<?> c, String field, Category cat, double min, double max, double step,
                            String unit, String desc, boolean placeholder, Tighten t) {
        String key = prefix + "." + field;
        PARAMS.put(key, new Param(key, Param.field(c, field), Param.fieldIsInt(c, field), Param.fieldIsBool(c, field),
                cat, min, max, step, unit, desc, placeholder, t));
    }
    private static void tur(String f, Category c, double min, double max, double step, String u, String d, boolean ph) {
        add("turret", TurretConfig.class, f, c, min, max, step, u, d, ph, Tighten.NONE);
    }
    private static void fly(String f, Category c, double min, double max, double step, String u, String d, boolean ph) {
        add("flywheel", FlywheelConfig.class, f, c, min, max, step, u, d, ph, Tighten.NONE);
    }
    private static void rob(String f, Category c, double min, double max, double step, String u, String d, boolean ph) {
        add("robot", RobotConfig.class, f, c, min, max, step, u, d, ph, Tighten.NONE);
    }
    private static void safe(String prefix, Class<?> cls, String f, double min, double max, double step, String u, String d, Tighten t) {
        add(prefix, cls, f, SAFE, min, max, step, u, d, false, t);
    }
    private static void target(String key, double dflt, boolean x, int idx, String desc) {
        Param.Accessor a = new Param.Accessor() {
            @Override public double get() { Storage.FieldPoint p = idx == 0 ? Storage.targetA : Storage.targetB; return x ? p.x : p.y; }
            @Override public void set(double v) {
                Storage.FieldPoint p = idx == 0 ? Storage.targetA : Storage.targetB;
                Storage.setTargetRaw(idx, x ? v : p.x, x ? p.y : v);
            }
        };
        PARAMS.put(key, new Param(key, a, false, false, HW, -400, 400, 1, "in", desc, true, Tighten.NONE));
    }

    static {
        // ---- turret: hardware calibration ----
        tur("SHAFT_TICKS_PER_REV", HW, 1, 100000, 1, "ticks", "encoder ticks per rev of the shaft the motor datasheet figures describe", true);
        tur("SHAFT_REVS_PER_TURRET_REV", HW, 0.01, 1000, 0.01, "rev/rev", "EXTERNAL reduction only (never include the motor's internal gearbox twice)", true);
        tur("ENCODER_SIGN", HW, -1, 1, 2, "", "+1/-1 so positive power increases the angle", true);
        tur("START_ANGLE_RAD", HW, -Math.PI, Math.PI, 0.01, "rad", "turret angle at OpMode init", false);
        tur("CONTINUOUS", HW, 0, 1, 1, "", "turret can spin without hard stops", true);
        add("turret", TurretConfig.class, "MIN_ANGLE_RAD", SAFE, -2 * Math.PI, 2 * Math.PI, 0.02, "rad", "reachable min (set from the measured hard stop)", true, Tighten.NONE);
        add("turret", TurretConfig.class, "MAX_ANGLE_RAD", SAFE, -2 * Math.PI, 2 * Math.PI, 0.02, "rad", "reachable max (set from the measured hard stop)", true, Tighten.NONE);
        tur("SHAFT_FREE_SPEED_RAD_S", HW, 0.1, 2000, 1, "rad/s", "datasheet free speed of the shaft", true);
        tur("SHAFT_STALL_TORQUE_NM", HW, 0.001, 100, 0.01, "N*m", "datasheet stall torque", true);
        tur("STALL_CURRENT_A", HW, 0.1, 100, 0.1, "A", "datasheet stall current", true);
        tur("LOAD_INERTIA_KG_M2", HW, 1e-6, 10, 0.001, "kg*m^2", "turret + payload inertia", true);
        tur("VISCOUS_FRICTION", HW, 0, 10, 0.001, "N*m/(rad/s)", "viscous friction", true);
        tur("ROBOT_VELOCITY_IS_FIELD_FRAME", HW, 0, 1, 1, "", "Pedro Follower.velocity() is field-frame (software verified; confirm raw Pinpoint output with LocalizationCal)", false);
        tur("STATIC_FRICTION_VOLTS", HW, 0, 6, 0.05, "V", "voltage that just creeps the turret", false);
        // ---- turret: controller / behaviour ----
        tur("MAX_ANGLE_ERROR_RAD", CTRL, 0.001, 1.5, 0.005, "rad", "LQR Q: angle error that costs as much as max volts", false);
        tur("MAX_VELOCITY_ERROR_RAD_S", CTRL, 0.01, 50, 0.05, "rad/s", "LQR Q: velocity error weight", false);
        tur("MAX_REFERENCE_VELOCITY_RAD_S", CTRL, 0.1, 50, 0.1, "rad/s", "reference slew limit on target switch", false);
        tur("VELOCITY_FILTER_ALPHA", CTRL, 0.01, 1, 0.05, "", "velocity low-pass (1 = none)", false);
        tur("INTEGRAL_GAIN", CTRL, 0, 50, 0.1, "V/(rad*s)", "integral action (0 = off)", false);
        tur("ALIGN_TOLERANCE_RAD", BEH, 0.001, 0.5, 0.005, "rad", "aligned if |error| below this", false);
        tur("ALIGN_VELOCITY_TOL_RAD_S", BEH, 0.01, 10, 0.05, "rad/s", "aligned if |velocity| below this", false);
        tur("MAX_POSE_AGE_MS", BEH, 10, 1000, 10, "ms", "pose older than this is stale", false);
        add("turret", TurretConfig.class, "HARDWARE_CONFIGURED", LOCK, 0, 1, 1, "", "INTERLOCK: allow AimTurret to power the turret", false, Tighten.NONE);
        safe("turret", TurretConfig.class, "MAX_CONTROL_VOLTS", 0.5, 12, 0.5, "V", "turret voltage cap", Tighten.LOWER);
        safe("turret", TurretConfig.class, "INTEGRAL_LIMIT_VOLTS", 0, 12, 0.1, "V", "integral clamp", Tighten.LOWER);
        safe("turret", TurretConfig.class, "LIMIT_MARGIN_RAD", 0, 1, 0.01, "rad", "power cut this close to a limit", Tighten.HIGHER);

        // ---- flywheel ----
        fly("MOTOR_COUNT", HW, 1, 4, 1, "", "motors driving the wheel", true);
        fly("SHAFT_FREE_SPEED_RAD_S", HW, 1, 5000, 1, "rad/s", "datasheet", true);
        fly("SHAFT_STALL_TORQUE_NM", HW, 0.001, 100, 0.005, "N*m", "datasheet", true);
        fly("STALL_CURRENT_A", HW, 0.1, 100, 0.1, "A", "datasheet", true);
        fly("SHAFT_TICKS_PER_REV", HW, 1, 100000, 1, "ticks", "encoder ticks per motor-shaft rev", true);
        fly("SHAFT_REVS_PER_FLYWHEEL_REV", HW, 0.05, 100, 0.05, "rev/rev", "1 = direct drive", true);
        fly("FLYWHEEL_INERTIA_KG_M2", HW, 1e-7, 1, 1e-5, "kg*m^2", "flywheel inertia", true);
        fly("VISCOUS_FRICTION", HW, 0, 1, 1e-6, "N*m/(rad/s)", "drag", true);
        fly("USE_SYSID_MODEL", HW, 0, 1, 1, "", "use kS/kV/kA instead of the physical model", false);
        fly("KS_VOLTS", HW, 0, 6, 0.05, "V", "sysid static friction", true);
        fly("KV_VOLTS_PER_TICK_S", HW, 1e-6, 1, 1e-5, "V/(tick/s)", "sysid velocity gain", true);
        fly("KA_VOLTS_PER_TICK_S2", HW, 1e-7, 1, 1e-4, "V/(tick/s^2)", "sysid acceleration gain", true);
        fly("MAX_VELOCITY_ERROR", CTRL, 1, 5000, 5, "ticks/s", "LQR Q weight: error that costs as much as max volts", false);
        fly("MODEL_STD", CTRL, 0.1, 5000, 5, "ticks/s", "Kalman model std-dev (bigger = trusts measurement more)", false);
        fly("MEASUREMENT_STD", CTRL, 0.1, 5000, 5, "ticks/s", "Kalman measurement std-dev", false);
        add("flywheel", FlywheelConfig.class, "MODEL_IDENTIFIED", LOCK, 0, 1, 1, "", "INTERLOCK: use LQR instead of SDK velocity PIDF", false, Tighten.NONE);
        safe("flywheel", FlywheelConfig.class, "MAX_CONTROL_VOLTS", 1, 12, 0.5, "V", "flywheel voltage cap", Tighten.LOWER);
        safe("flywheel", FlywheelConfig.class, "MAX_PLAUSIBLE_VELOCITY", 100, 20000, 100, "ticks/s", "sanity bound on measured velocity", Tighten.LOWER);

        // ---- robot behaviour ----
        rob("FLYWHEEL_TOLERANCE", BEH, 1, 1000, 5, "ticks/s", "ready band", false);
        rob("FLYWHEEL_STABLE_MS", BEH, 0, 5000, 10, "ms", "time in band before ready", false);
        rob("FLYWHEEL_SPINUP_TIMEOUT_MS", BEH, 100, 30000, 100, "ms", "spin-up timeout", false);
        rob("STOPPER_CLOSED", HW, 0, 1, 0.01, "servo", "closed position", true);
        rob("STOPPER_OPEN", HW, 0, 1, 0.01, "servo", "open position", true);
        rob("STOPPER_SETTLE_MS", BEH, 0, 2000, 10, "ms", "opening settle (assumed, no servo feedback)", true);
        rob("STOPPER_CLOSE_SETTLE_MS", BEH, 0, 2000, 10, "ms", "closing settle (assumed)", true);
        rob("FEED_POWER", CTRL, 0.05, 1, 0.05, "", "feed motor power", false);
        rob("INTAKE_POWER", CTRL, 0.05, 1, 0.05, "", "intake motor power", false);
        rob("READY_TIMEOUT_MS", BEH, 100, 30000, 100, "ms", "wait for the shooting window", false);
        rob("SHOT_TIMEOUT_MS", BEH, 100, 10000, 50, "ms", "feed -> RPM drop timeout", false);
        rob("RECOVER_TIMEOUT_MS", BEH, 100, 20000, 100, "ms", "recovery timeout", false);
        rob("SHOT_DROP_THRESHOLD", BEH, 5, 2000, 5, "ticks/s", "RPM drop that counts as a shot", false);
        rob("DROP_CONFIRM_LOOPS", BEH, 1, 20, 1, "loops", "consecutive loops of drop", false);
        rob("DETECT_INTAKE_JAM", BEH, 0, 1, 1, "", "enable current-based jam detect", false);
        rob("INTAKE_JAM_AMPS", BEH, 0.5, 30, 0.5, "A", "jam current", true);
        rob("INTAKE_JAM_MS", BEH, 50, 5000, 50, "ms", "jam duration", true);
        rob("MAX_SHOOT_SPEED", BEH, 0.5, 200, 1, "in/s", "max robot speed for a shot", true);
        rob("FLYWHEEL_VELOCITY_A", BEH, 100, 20000, 25, "ticks/s", "flywheel target for target A", true);
        rob("FLYWHEEL_VELOCITY_B", BEH, 100, 20000, 25, "ticks/s", "flywheel target for target B", true);
        rob("REQUIRE_LOCALIZATION", BEH, 0, 1, 1, "", "readiness needs fresh pose", false);
        rob("REQUIRE_TURRET_ALIGNED", LOCK, 0, 1, 1, "", "readiness needs turret aligned", false);
        rob("SHOOT_ZONE_ENABLED", BEH, 0, 1, 1, "", "readiness needs robot in zone", false);
        rob("SHOOT_ZONE_MIN_X", BEH, -400, 400, 1, "in", "zone", true);
        rob("SHOOT_ZONE_MAX_X", BEH, -400, 400, 1, "in", "zone", true);
        rob("SHOOT_ZONE_MIN_Y", BEH, -400, 400, 1, "in", "zone", true);
        rob("SHOOT_ZONE_MAX_Y", BEH, -400, 400, 1, "in", "zone", true);
        safe("robot", RobotConfig.class, "MAX_FLYWHEEL_VELOCITY", 100, 20000, 100, "ticks/s", "upper bound for any commanded target", Tighten.LOWER);
        safe("robot", RobotConfig.class, "MAX_BALLS", 1, 10, 1, "balls", "max balls per shoot command", Tighten.LOWER);
        // ---- calibration procedure settings ----
        add("cal", CalSettings.class, "FLYWHEEL_STEP_HOLD_S", BEH, 2, 20, 1, "s", "flywheel step-response length", false, Tighten.NONE);
        add("cal", CalSettings.class, "DISTURBANCE_MS", BEH, 20, 1000, 20, "ms", "length of the injected power cut", false, Tighten.NONE);
        add("cal", CalSettings.class, "TURRET_STEP_DEG", BEH, 5, 60, 5, "deg", "turret step size", false, Tighten.NONE);
        add("cal", CalSettings.class, "TURRET_KNOWN_ANGLE_DEG", BEH, 30, 360, 15, "deg", "angle for hand-rotation ticks/rev measurement", false, Tighten.NONE);
        add("cal", CalSettings.class, "TURRET_AIM_SECONDS", BEH, 2, 60, 1, "s", "aim accuracy run length", false, Tighten.NONE);
        add("cal", CalSettings.class, "MONITOR_SECONDS", BEH, 2, 90, 2, "s", "shooting-window monitor length", false, Tighten.NONE);
        add("cal", CalSettings.class, "STAGE_BALLS", BEH, 1, 10, 1, "balls", "balls for shooting stages", false, Tighten.NONE);
        add("cal", CalSettings.class, "WINDOW_MS", BEH, 100, 30000, 250, "ms", "per-ball wait for the window in stages", false, Tighten.NONE);
        add("cal", CalSettings.class, "STAGE_SECONDS", BEH, 3, 90, 1, "s", "stage time limit", false, Tighten.NONE);
        for (String n : new String[]{"PATH_START_X", "PATH_START_Y", "PATH_END_X", "PATH_END_Y"})
            add("cal", CalSettings.class, n, HW, -400, 400, 1, "in", "test path point (field measurement; unset by default)", true, Tighten.NONE);
        for (String n : new String[]{"PATH_START_HEADING_DEG", "PATH_END_HEADING_DEG"})
            add("cal", CalSettings.class, n, HW, -360, 360, 5, "deg", "test path heading (field measurement; unset by default)", true, Tighten.NONE);
        // ---- targets ----
        target("target.A_X", Double.NaN, true, 0, "target A x");
        target("target.A_Y", Double.NaN, false, 0, "target A y");
        target("target.B_X", Double.NaN, true, 1, "target B x");
        target("target.B_Y", Double.NaN, false, 1, "target B y");

        for (Map.Entry<String, Param> e : PARAMS.entrySet()) DEFAULTS.put(e.getKey(), e.getValue().get());
    }

    public static Param get(String key) { return PARAMS.get(key); }
    public static Map<String, Param> all() { return Collections.unmodifiableMap(PARAMS); }
    public static double defaultOf(String key) { return DEFAULTS.get(key); }
    public static boolean has(String key) { return PARAMS.containsKey(key); }

    /** Live values of every parameter. */
    public static Map<String, Double> snapshot() {
        Map<String, Double> m = new TreeMap<>();
        for (Param p : PARAMS.values()) m.put(p.key, p.get());
        return m;
    }

    /** Restores every parameter to its compiled default (test isolation / "reject"). */
    public static void resetToDefaults() {
        for (Param p : PARAMS.values()) forceSet(p, DEFAULTS.get(p.key));
    }

    /** Sets live values from overrides after validating each. Throws on the first invalid entry; nothing is half-applied. */
    public static void apply(Map<String, Double> overrides) {
        Map<String, Double> before = snapshot();
        try {
            for (Map.Entry<String, Double> e : overrides.entrySet()) {
                Param p = PARAMS.get(e.getKey());
                if (p == null) throw new IllegalArgumentException("unknown parameter " + e.getKey());
                if (!Double.isNaN(e.getValue()) || !Double.isNaN(DEFAULTS.get(e.getKey()))) p.set(e.getValue());
            }
        } catch (RuntimeException ex) {
            restore(before);
            throw ex;
        }
    }

    public static void restore(Map<String, Double> snap) {
        for (Map.Entry<String, Double> e : snap.entrySet()) forceSet(PARAMS.get(e.getKey()), e.getValue());
    }

    /** Runs {@code body} with overrides applied on top of the compiled defaults, then restores the previous values. */
    public static <T> T with(Map<String, Double> overrides, Supplier<T> body) {
        Map<String, Double> before = snapshot();
        try {
            resetToDefaults();
            apply(overrides);
            return body.get();
        } finally {
            restore(before);
        }
    }

    /** Bypasses range checks: only for restoring values that were already live (may be NaN defaults). */
    private static void forceSet(Param p, double v) {
        try {
            java.lang.reflect.Method m = Param.class.getDeclaredMethod("rawSet", double.class);
            m.setAccessible(true);
            m.invoke(p, v);
        } catch (Exception e) { throw new IllegalStateException(e); }
    }

    public static List<Param> adjustable() {
        List<Param> l = new ArrayList<>();
        for (Param p : PARAMS.values()) if (p.manuallyAdjustable()) l.add(p);
        return l;
    }
}
