package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.info;
import static org.firstinspires.ftc.teamcode.T.near;
import static org.firstinspires.ftc.teamcode.commands.SimSupport.*;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.calibration.ProcedureResult;
import org.firstinspires.ftc.teamcode.calibration.TurretCalLogic;
import org.firstinspires.ftc.teamcode.calibration.TurretProcedures;
import org.firstinspires.ftc.teamcode.control.TurretConfig;
import org.firstinspires.ftc.teamcode.hardware.Storage;

import java.util.ArrayList;

/** Turret calibration procedures against the simulator. SIMULATION ONLY. */
public final class TurretCalibrationTests {
    private TurretCalibrationTests() {}

    private static SimCalContext.Operator scripted(final java.util.function.BiFunction<String, SimRobot, Boolean> conf) {
        return new SimCalContext.Operator() {
            @Override public boolean confirm(String p, SimRobot s) { return conf.apply(p, s); }
            @Override public double value(String p, double i, SimRobot s) { return i; }
        };
    }

    public static void run() {
        sim("cal turret logic: direction verdicts and the external-ratio arithmetic that avoids double counting", s -> {
            ArrayList<String> n = new ArrayList<>();
            check(TurretCalLogic.directionSuggestion(120, true, n).get("turret.ENCODER_SIGN") == 1.0, "ticks up + CCW -> +1");
            check(TurretCalLogic.directionSuggestion(-120, true, n).get("turret.ENCODER_SIGN") == -1.0, "ticks down + CCW -> -1");
            n.clear();
            check(TurretCalLogic.directionSuggestion(120, false, n).isEmpty() && n.get(0).contains("RobotHardware"), "physical CW -> reverse the motor direction, no sign suggested");
            n.clear();
            check(TurretCalLogic.directionSuggestion(2, true, n).isEmpty() && n.get(0).contains("barely moved"), "no encoder motion");
            // 90 deg = 750 ticks -> 3000 ticks per turret rev; shaft encoder 1000 ticks/rev -> external ratio 3, NOT 3 * internal gearbox
            double[] ms = TurretCalLogic.ticksPerTurretRev(new double[]{750, 752, 748}, 90);
            near(3000, ms[0], 1, "ticks per turret rev");
            near(3.0, TurretCalLogic.externalRatio(ms[0], 1000), 0.01, "external ratio");
            near(ms[0], 1000 * TurretCalLogic.externalRatio(ms[0], 1000), 1e-9, "shaftTicks * ratio reproduces the measured value exactly (no double count)");
        });

        sim("cal turret direction: reports the encoder sign from the simulated motor", s -> {
            SimCalContext c = new SimCalContext(s);
            ProcedureResult r = TurretProcedures.encoderDirection().run(c);
            check(!r.refused && !r.aborted, "ran " + r.notes);
            check(r.metrics.get("delta_ticks") > 5, "encoder moved: " + r.metrics.get("delta_ticks"));
            check(r.suggestions.get("turret.ENCODER_SIGN") == 1.0, "sign +1");
            near(0, s.turretM.power, 0, "motor stopped afterwards");
            double maxPower = 0; for (double p : r.log.column("power")) maxPower = Math.max(maxPower, Math.abs(p));
            check(maxPower <= 0.151, "nudge power is small: " + maxPower);
            // wired the other way round
            TurretConfig.ENCODER_SIGN = -1;
            ProcedureResult r2 = TurretProcedures.encoderDirection().run(new SimCalContext(s));
            check(r2.suggestions.get("turret.ENCODER_SIGN") == -1.0, "sign -1 detected");
            // operator saw it turn the wrong way physically
            TurretConfig.ENCODER_SIGN = 1;
            SimCalContext cw = new SimCalContext(s);
            cw.operator = scripted((p, x) -> !p.contains("COUNTER-CLOCKWISE"));
            ProcedureResult r3 = TurretProcedures.encoderDirection().run(cw);
            check(r3.suggestions.isEmpty() && String.join(" ", r3.notes).contains("CLOCKWISE"), "CW -> reverse motor, no suggestion");
            // operator declines the safety question
            SimCalContext no = new SimCalContext(s);
            no.operator = scripted((p, x) -> false);
            long w = s.turretM.powerWrites;
            check(TurretProcedures.encoderDirection().run(no).aborted && s.turretM.powerWrites == w, "declined: motor never powered");
        });

        sim("cal turret ticks/rev: hand rotations give the measured conversion and the encoder consistency", s -> {
            // "real" turret: 1.5% fewer ticks per revolution than configured
            final double scale = 0.985;
            SimCalContext c = new SimCalContext(s);
            final double[] last = {0};
            c.operator = scripted((p, x) -> {
                if (p.contains("mark 0")) { x.theta = 0; x.step(10); }
                if (p.contains("Rotate it BY HAND")) { x.theta = Math.toRadians(90) / scale; x.step(10); }   // encoder shows 'scale' of expected
                return true;
            });
            ProcedureResult r = TurretProcedures.ticksPerRev(4, 90).run(c);
            check(!r.refused && !r.aborted, "ran " + r.notes);
            double configured = TurretConfig.SHAFT_TICKS_PER_REV * TurretConfig.SHAFT_REVS_PER_TURRET_REV;
            info("measured ticks/turret-rev " + r.metrics.get("ticks_per_turret_rev") + " vs configured " + configured);
            near(configured / scale, r.metrics.get("ticks_per_turret_rev"), 2.0, "measured conversion");
            check(r.metrics.get("encoder_spread_pct") < 0.2, "consistent repeats");
            double sug = r.suggestions.get("turret.SHAFT_REVS_PER_TURRET_REV");
            near(TurretConfig.SHAFT_REVS_PER_TURRET_REV / scale, sug, 0.005, "external ratio suggestion");
            near(r.metrics.get("ticks_per_turret_rev"), TurretConfig.SHAFT_TICKS_PER_REV * sug, 1e-6, "no double counting: shaft ticks x ratio == measured");
            check(!r.suggestions.containsKey("turret.SHAFT_TICKS_PER_REV"), "shaft encoder resolution is not touched");
            check(s.turretM.power == 0, "motor unpowered during hand rotation");
            check(TurretProcedures.ticksPerRev(1, 90).run(new SimCalContext(s)).refused, "needs >= 2 repeats");
        });

        sim("cal turret limits: measured hard stops become inset limits that pass validation", s -> {
            SimCalContext c = new SimCalContext(s);
            c.operator = scripted((p, x) -> {
                if (p.contains("MINIMUM")) { x.theta = -1.70; x.step(10); }
                if (p.contains("MAXIMUM")) { x.theta = 1.60; x.step(10); }
                return true;
            });
            ProcedureResult r = TurretProcedures.limits().run(c);
            near(-1.70 + Math.toRadians(5), r.suggestions.get("turret.MIN_ANGLE_RAD"), 0.01, "min");
            near(1.60 - Math.toRadians(5), r.suggestions.get("turret.MAX_ANGLE_RAD"), 0.01, "max");
            near(Math.toDegrees(3.30), r.metrics.get("range_deg"), 0.5, "range");
            java.util.Map<String, Double> cand = new java.util.TreeMap<>(r.suggestions);
            check(org.firstinspires.ftc.teamcode.tuning.ConfigValidator.validate(cand).isEmpty(), "limits validate");
            // reversed entry order is handled
            SimCalContext rev = new SimCalContext(s);
            rev.operator = scripted((p, x) -> { if (p.contains("MINIMUM")) { x.theta = 1.6; x.step(10); } if (p.contains("MAXIMUM")) { x.theta = -1.7; x.step(10); } return true; });
            check(TurretProcedures.limits().run(rev).suggestions.get("turret.MIN_ANGLE_RAD") < 0, "swapped order corrected");
        });

        sim("cal turret step response: production controller, voltage cap respected, metrics produced", s -> {
            SimCalContext c = new SimCalContext(s);
            ProcedureResult r = TurretProcedures.stepResponse(20, 2.5).run(c);
            check(!r.refused && !r.aborted, "ran " + r.notes);
            double cap = 3.0 / 12.0;
            check(r.metrics.get("peak_power") <= cap + 1e-9, "power capped at " + cap + ": " + r.metrics.get("peak_power"));
            check(r.metrics.get("limit_violations") == 0, "no limit violations");
            info("rise " + r.metrics.get("rise_time_s") + " s, overshoot " + r.metrics.get("overshoot_pct") + " %, tracking rms " + Math.toDegrees(r.metrics.get("tracking_rms")) + " deg, saturation " + r.metrics.get("saturation_fraction"));
            check(r.metrics.get("settling_time_s") >= 0 && r.metrics.get("tracking_rms") < Math.toRadians(25), "settles");
            near(0, s.turretM.power, 0, "stopped");
            check(String.join(" ", c.prompts).contains("PLACEHOLDER"), "operator warned that placeholders are in use");
            check(TurretProcedures.stepResponse(80, 2).run(new SimCalContext(s)).refused, "step > 60 deg refused");
            check(TurretProcedures.stepResponse(0, 2).run(new SimCalContext(s)).refused, "zero step refused");
        });

        sim("cal turret step response: refuses near a limit and declines placeholders on request", s -> {
            s.theta = 1.5;   // near the +90 deg (1.571 rad) limit
            s.step(10);
            long w = s.turretM.powerWrites;
            ProcedureResult r = TurretProcedures.stepResponse(20, 2).run(new SimCalContext(s));
            check(r.refused && String.join(" ", r.notes).contains("limits") && s.turretM.powerWrites == w, "refused: " + r.notes);
            s.theta = 0; s.step(10);
            SimCalContext c = new SimCalContext(s);
            c.operator = scripted((p, x) -> !p.contains("PLACEHOLDER"));
            ProcedureResult d = TurretProcedures.stepResponse(20, 2).run(c);
            check(d.aborted && s.turretM.powerWrites == w, "operator declined the placeholder warning: motor never driven");
        });

        sim("cal turret aim accuracy: production AimTurret, robot turned by hand, target switch, config restored", s -> {
            Storage.setTarget(0, 60, 0); Storage.setTarget(1, 60, 30);
            s.follower.pose = new Pose(0, 0, 0);
            SimCalContext c = new SimCalContext(s);
            final int[] i = {0};
            c.onLoop = x -> { i[0]++; x.follower.pose = new Pose(0, 0, Math.toRadians(15) * (i[0] * 0.01)); };   // 15 deg/s by hand
            double oldV = TurretConfig.MAX_CONTROL_VOLTS;
            ProcedureResult r = TurretProcedures.aimAccuracy(8, 4).run(c);
            check(!r.refused && !r.aborted, "ran " + r.notes);
            info("aim rms " + r.metrics.get("aim_rms_deg") + " deg, max " + r.metrics.get("aim_max_deg") + ", aligned " + r.metrics.get("aligned_fraction") + ", realign " + r.metrics.get("realign_time_s"));
            check(r.metrics.get("aim_rms_deg") < 6, "tracks while turning");
            check(r.metrics.get("aligned_fraction") > 0.5, "mostly aligned");
            check(r.metrics.get("realign_time_s") > 0 && r.metrics.get("realign_time_s") < 3, "re-aims after the switch");
            check(!TurretConfig.HARDWARE_CONFIGURED && TurretConfig.MAX_CONTROL_VOLTS == oldV, "temporary overrides restored");
            near(0, s.turretM.power, 0, "stopped");
            Storage.targetA = new Storage.FieldPoint(Double.NaN, Double.NaN); Storage.selectTarget(0);
            check(TurretProcedures.aimAccuracy(5, 0).run(new SimCalContext(s)).refused, "no target set: refused");
        });

        sim("cal turret sysid: refuses with placeholders, identifies inertia/friction from pulses when calibrated", s -> {
            check(TurretProcedures.sysId(1.5).run(new SimCalContext(s)).refused, "placeholders in use -> refused");
            // calibrated-looking values, then rebuild the simulated plant from them
            TurretConfig.SHAFT_TICKS_PER_REV = 1001; TurretConfig.SHAFT_REVS_PER_TURRET_REV = 3.1;
            TurretConfig.LOAD_INERTIA_KG_M2 = 0.05; TurretConfig.VISCOUS_FRICTION = 0.02;
            s.refreshPlants();
            SimCalContext c = new SimCalContext(s);
            ProcedureResult r = TurretProcedures.sysId(1.5).run(c);
            check(!r.refused && !r.aborted, "ran " + r.notes);
            info("notes " + r.notes + " metrics " + r.metrics);
            info("fit r2 " + r.metrics.get("fit_r2") + "  J suggestion " + r.suggestions.get("turret.LOAD_INERTIA_KG_M2") + " (true 0.05)");
            near(0.05, r.suggestions.get("turret.LOAD_INERTIA_KG_M2"), 0.05 * 0.2, "inertia within 20%");
            near(0, s.turretM.power, 0, "stopped");
            check(TurretProcedures.sysId(5.0).run(new SimCalContext(s)).refused, "voltage above the cap refused");
        });

        sim("cal turret zero: reports the zero error of the forward mark", s -> {
            SimCalContext c = new SimCalContext(s);
            c.operator = new SimCalContext.Operator() {
                @Override public boolean confirm(String p, SimRobot x) { x.theta = Math.toRadians(2.5); x.step(10); return true; }
                @Override public double value(String p, double i, SimRobot x) { return 0; }
            };
            ProcedureResult r = TurretProcedures.zeroCheck().run(c);
            near(2.5, r.metrics.get("zero_error_deg"), 0.1, "zero error");
            check(String.join(" ", r.notes).contains("BEFORE pressing INIT"), "tells the operator how to fix it");
        });
    }
}
