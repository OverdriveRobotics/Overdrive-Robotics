package org.firstinspires.ftc.teamcode.tuning;

import org.firstinspires.ftc.teamcode.commands.RobotConfig;
import org.firstinspires.ftc.teamcode.control.FlywheelConfig;
import org.firstinspires.ftc.teamcode.control.FlywheelLqrController;
import org.firstinspires.ftc.teamcode.control.TurretConfig;
import org.firstinspires.ftc.teamcode.control.TurretStateSpaceController;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Validates a set of OVERRIDES (parameter -> value, on top of compiled defaults). Checks, in order: unknown keys,
 * per-parameter range/type, SAFETY tighten-only rules, cross-parameter consistency, interlock prerequisites
 * (the dependent hardware values must be explicitly present, never inherited from a placeholder default) and finally
 * that the real controllers can be constructed from the resulting configuration. Never mutates live state.
 */
public final class ConfigValidator {
    private ConfigValidator() {}

    public static List<String> validate(Map<String, Double> o) {
        List<String> err = new ArrayList<>();
        for (Map.Entry<String, Double> e : o.entrySet()) {
            Param p = ParamRegistry.get(e.getKey());
            if (p == null) { err.add("unknown parameter '" + e.getKey() + "'"); continue; }
            double v = e.getValue();
            if (Double.isNaN(v) && Double.isNaN(ParamRegistry.defaultOf(p.key))) continue;
            String c = p.check(v);
            if (c != null) { err.add(p.key + ": " + c); continue; }
            if (p.category == Param.Category.SAFETY) {
                double d = ParamRegistry.defaultOf(p.key);
                if (p.tighten == Param.Tighten.LOWER && v > d + 1e-12) err.add(p.key + ": safety limit may only be lowered (default " + d + ", got " + v + ")");
                if (p.tighten == Param.Tighten.HIGHER && v < d - 1e-12) err.add(p.key + ": safety margin may only be increased (default " + d + ", got " + v + ")");
            }
        }
        if (!err.isEmpty()) return err;

        ParamRegistry.with(o, () -> { crossChecks(o, err); return null; });
        return err;
    }

    private static boolean has(Map<String, Double> o, String k) { return o.containsKey(k) && !Double.isNaN(o.get(k)); }
    private static boolean on(String k) { return ParamRegistry.get(k).get() != 0; }
    private static double val(String k) { return ParamRegistry.get(k).get(); }

    private static void crossChecks(Map<String, Double> o, List<String> err) {
        double sign = TurretConfig.ENCODER_SIGN;
        if (sign != 1 && sign != -1) err.add("turret.ENCODER_SIGN must be +1 or -1");
        if (!TurretConfig.CONTINUOUS && !(TurretConfig.MIN_ANGLE_RAD < TurretConfig.MAX_ANGLE_RAD))
            err.add("turret.MIN_ANGLE_RAD must be below MAX_ANGLE_RAD");
        if (TurretConfig.START_ANGLE_RAD < TurretConfig.MIN_ANGLE_RAD - 1e-9 && !TurretConfig.CONTINUOUS
                || TurretConfig.START_ANGLE_RAD > TurretConfig.MAX_ANGLE_RAD + 1e-9 && !TurretConfig.CONTINUOUS)
            err.add("turret.START_ANGLE_RAD lies outside the turret limits");
        if (RobotConfig.STOPPER_OPEN == RobotConfig.STOPPER_CLOSED) err.add("robot.STOPPER_OPEN and STOPPER_CLOSED must differ");
        if (RobotConfig.FLYWHEEL_VELOCITY_A > RobotConfig.MAX_FLYWHEEL_VELOCITY || RobotConfig.FLYWHEEL_VELOCITY_B > RobotConfig.MAX_FLYWHEEL_VELOCITY)
            err.add("flywheel target velocity exceeds robot.MAX_FLYWHEEL_VELOCITY");
        if (RobotConfig.SHOOT_ZONE_ENABLED) {
            if (!(RobotConfig.SHOOT_ZONE_MIN_X < RobotConfig.SHOOT_ZONE_MAX_X) || !(RobotConfig.SHOOT_ZONE_MIN_Y < RobotConfig.SHOOT_ZONE_MAX_Y))
                err.add("robot.SHOOT_ZONE_ENABLED needs finite zone bounds with min < max");
        }
        // ---- interlocks ----
        if (on("turret.HARDWARE_CONFIGURED")) {
            String[] need = {"SHAFT_TICKS_PER_REV", "SHAFT_REVS_PER_TURRET_REV", "ENCODER_SIGN", "MIN_ANGLE_RAD", "MAX_ANGLE_RAD",
                    "SHAFT_FREE_SPEED_RAD_S", "SHAFT_STALL_TORQUE_NM", "STALL_CURRENT_A", "LOAD_INERTIA_KG_M2", "VISCOUS_FRICTION"};
            for (String n : need) if (!has(o, "turret." + n))
                err.add("turret.HARDWARE_CONFIGURED=true requires turret." + n + " to be explicitly calibrated (it is still a placeholder)");
            tryBuildTurret(err);
        }
        if (on("flywheel.MODEL_IDENTIFIED")) {
            String[] need = on("flywheel.USE_SYSID_MODEL")
                    ? new String[]{"KV_VOLTS_PER_TICK_S", "KA_VOLTS_PER_TICK_S2", "KS_VOLTS"}
                    : new String[]{"MOTOR_COUNT", "SHAFT_FREE_SPEED_RAD_S", "SHAFT_STALL_TORQUE_NM", "STALL_CURRENT_A",
                    "SHAFT_TICKS_PER_REV", "SHAFT_REVS_PER_FLYWHEEL_REV", "FLYWHEEL_INERTIA_KG_M2", "VISCOUS_FRICTION"};
            for (String n : need) if (!has(o, "flywheel." + n))
                err.add("flywheel.MODEL_IDENTIFIED=true requires flywheel." + n + " to be explicitly identified (placeholder otherwise)");
            tryBuildFlywheel(err);
        }
        if (on("robot.REQUIRE_TURRET_ALIGNED")) {
            if (!on("turret.HARDWARE_CONFIGURED")) err.add("robot.REQUIRE_TURRET_ALIGNED=true requires turret.HARDWARE_CONFIGURED=true");
            if (!(has(o, "target.A_X") && has(o, "target.A_Y")) && !(has(o, "target.B_X") && has(o, "target.B_Y")))
                err.add("robot.REQUIRE_TURRET_ALIGNED=true requires at least one explicit target point");
        }
    }

    private static void tryBuildTurret(List<String> err) {
        try {
            new TurretStateSpaceController(TurretStateSpaceController.Params.fromConfig());
        } catch (RuntimeException e) { err.add("turret controller cannot be designed from these values: " + e.getMessage()); }
    }

    private static void tryBuildFlywheel(List<String> err) {
        try {
            new FlywheelLqrController(FlywheelLqrController.Params.fromConfig());
        } catch (RuntimeException e) { err.add("flywheel controller cannot be designed from these values: " + e.getMessage()); }
    }

    /** For diagnostics: would the controllers build under these overrides (without enabling interlocks)? */
    public static List<String> controllersBuild(Map<String, Double> o) {
        List<String> err = new ArrayList<>();
        ParamRegistry.with(o, () -> { tryBuildTurret(err); tryBuildFlywheel(err); return null; });
        return err;
    }
}
