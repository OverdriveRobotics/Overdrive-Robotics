package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.info;
import static org.firstinspires.ftc.teamcode.T.near;
import static org.firstinspires.ftc.teamcode.commands.SimSupport.*;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.calibration.FlywheelProcedures;
import org.firstinspires.ftc.teamcode.calibration.ProcedureResult;
import org.firstinspires.ftc.teamcode.control.FlywheelConfig;
import org.firstinspires.ftc.teamcode.hardware.Storage;

/** Flywheel calibration procedures run against the simulator. SIMULATION ONLY. */
public final class FlywheelCalibrationTests {
    private FlywheelCalibrationTests() {}

    public static void run() {
        sim("cal flywheel step: metrics, log and safe end (SDK velocity path)", s -> {
            SimCalContext c = new SimCalContext(s);
            ProcedureResult r = FlywheelProcedures.stepResponse(1800, 6).run(c);
            check(!r.refused && !r.aborted, "ran: " + r.notes);
            info("settling " + r.metrics.get("settling_time_s") + " s, ready after " + r.metrics.get("readiness_delay_s") + " s, ss err " + r.metrics.get("steady_state_error"));
            check(r.metrics.get("settling_time_s") > 0.3 && r.metrics.get("settling_time_s") < 5, "settling measured");
            check(r.metrics.get("readiness_delay_s") >= r.metrics.get("settling_time_s") - 0.05, "ready no earlier than settled");
            check(Math.abs(r.metrics.get("steady_state_error")) < 60, "steady-state error within the tolerance band");
            near(0, r.metrics.get("power_range_violations"), 0, "no power out of range");
            check(r.log != null && r.log.size() > 400 && r.log.columns.length == 8, "timestamped log");
            near(0, Storage.flywheelTargetVelocity, 0, "flywheel stopped afterwards");
            check("SIMULATION".equals(c.mode()), "labelled");
        });

        sim("cal flywheel step: LQR path reports saturation and respects limits", s -> {
            FlywheelConfig.MODEL_IDENTIFIED = true;
            SimCalContext c = new SimCalContext(s);
            ProcedureResult r = FlywheelProcedures.stepResponse(1800, 7).run(c);
            check(!r.refused && !r.aborted, "ran: " + r.notes);
            check(r.notes.contains("control path: LQR"), "labelled control path: " + r.notes);
            check(r.metrics.get("saturation_fraction") >= 0 && r.metrics.get("saturation_fraction") < 0.5, "saturation fraction reported");
            near(0, r.metrics.get("power_range_violations"), 0, "power stays in [0,1]");
            check(r.metrics.get("battery_min_v") > 11 && r.metrics.get("battery_min_v") <= 12.0, "battery recorded");
            check(Math.abs(r.metrics.get("steady_state_error")) < 40, "tracks: " + r.metrics.get("steady_state_error"));
        });

        sim("cal flywheel: refuses bad targets, missing hardware and low battery before moving anything", s -> {
            double[] bad = {Double.NaN, -10, 0, 1e9};
            for (double t : bad) check(FlywheelProcedures.stepResponse(t, 5).run(new SimCalContext(s)).refused, "refused target " + t);
            s.vbat = 9.5;
            ProcedureResult r = FlywheelProcedures.stepResponse(1800, 5).run(new SimCalContext(s));
            check(r.refused && String.join(" ", r.notes).contains("battery"), "low battery: " + r.notes);
            s.vbat = 12;
            s.robot.shooterLeft = null;
            check(FlywheelProcedures.stepResponse(1800, 5).run(new SimCalContext(s)).refused, "missing motor");
            check(s.shooterR.powerWrites == 0 && s.shooterR.velCalls == 0, "nothing was commanded");
            check(FlywheelProcedures.sysId(new double[]{0.3}, 3, 3).run(new SimCalContext(s)).refused, "sysid refused too");
        });

        sim("cal flywheel sysid: identifies the simulated plant from noisy open-loop data, never applies it", s -> {
            s.noiseStd = 6;
            SimCalContext c = new SimCalContext(s);
            double[] powers = {0.35, 0.5, 0.65};
            ProcedureResult r = FlywheelProcedures.sysId(powers, 3, 4).run(c);
            check(!r.refused && !r.aborted, "ran: " + r.notes);
            double a = s.flywheelA(), b = s.flywheelB();
            double kvTrue = a / b, kaTrue = 1 / b;
            info(String.format("fit kV %.5f (true %.5f)  kA %.5f (true %.5f)  r2 %.3f", r.metrics.get("fit_kV"), kvTrue, r.metrics.get("fit_kA"), kaTrue, r.metrics.get("fit_r2")));
            near(kvTrue, r.suggestions.get("flywheel.KV_VOLTS_PER_TICK_S"), kvTrue * 0.10, "kV");
            near(kaTrue, r.suggestions.get("flywheel.KA_VOLTS_PER_TICK_S2"), kaTrue * 0.25, "kA");
            check(r.suggestions.get("flywheel.USE_SYSID_MODEL") == 1.0, "proposes sysid model");
            check(!FlywheelConfig.USE_SYSID_MODEL, "suggestion not applied to live config");
            check(r.metrics.get("peak_velocity") > 800, "wheel actually spun");
            check(s.shooterL.power == 0 && s.shooterL.mode == DcMotor.RunMode.RUN_USING_ENCODER, "motors handed back clean");
            double maxP = 0; for (double p : r.log.column("power")) maxP = Math.max(maxP, p);
            check(maxP <= 0.65 + 1e-9, "never above the requested power");
        });

        sim("cal flywheel sysid: power cap, operator decline and abort leave the motors off", s -> {
            check(FlywheelProcedures.sysId(new double[]{0.95}, 3, 3).run(new SimCalContext(s)).refused, "power above the calibration cap refused");
            SimCalContext decline = new SimCalContext(s);
            decline.operator = new SimCalContext.Operator() {
                @Override public boolean confirm(String p, SimRobot x) { return false; }
                @Override public double value(String p, double i, SimRobot x) { return i; }
            };
            ProcedureResult r = FlywheelProcedures.sysId(new double[]{0.4}, 3, 3).run(decline);
            check(r.aborted && s.shooterL.powerWrites == 0, "declined: no motor writes");
            SimCalContext ab = new SimCalContext(s);
            ab.abortAtSec = ab.timeSec() + 4.5;
            FlywheelProcedures.sysId(new double[]{0.4, 0.5}, 3, 3).run(ab);
            near(0, s.shooterL.power, 0, "aborted mid-run: power 0");
            near(0, s.shooterR.power, 0, "aborted mid-run: power 0 (right)");
        });

        sim("cal flywheel disturbance: controlled power cut, recovery time and readiness loss measured", s -> {
            SimCalContext c = new SimCalContext(s);
            ProcedureResult r = FlywheelProcedures.disturbance(1800, 200).run(c);
            check(!r.refused && !r.aborted, "ran: " + r.notes);
            info("drop " + r.metrics.get("disturbance_drop") + " ticks/s, recovered after " + r.metrics.get("recovery_time_s")
                    + " s, ready again after " + r.metrics.get("ready_again_after_s") + " s");
            check(r.metrics.get("disturbance_drop") > 150, "a real dip was produced");
            check(r.metrics.get("ready_lost_after_s") < 0.1, "readiness dropped at once");
            check(r.metrics.get("recovery_time_s") > 0 && r.metrics.get("ready_again_after_s") >= r.metrics.get("recovery_time_s"), "recovery then ready");
            near(0, Storage.flywheelDisturbanceMs, 0, "hook consumed/cleared");
            check(FlywheelProcedures.disturbance(1800, 0).run(new SimCalContext(s)).refused, "zero-length disturbance refused");
        });

        sim("cal flywheel scaling: tachometer reading yields a ticks-per-rev estimate and a suggestion", s -> {
            SimCalContext c = new SimCalContext(s);
            final double trueTpr = 28 * 1.2;   // the "real" encoder differs from the placeholder by 20%
            c.operator = new SimCalContext.Operator() {
                @Override public boolean confirm(String p, SimRobot x) { return true; }
                @Override public double value(String p, double i, SimRobot x) {
                    return x.omega / trueTpr * 60.0 / FlywheelConfig.SHAFT_REVS_PER_FLYWHEEL_REV;   // flywheel rpm
                }
            };
            ProcedureResult r = FlywheelProcedures.scaling(0.4).run(c);
            check(!r.refused && !r.aborted, "ran: " + r.notes);
            near(trueTpr, r.metrics.get("ticks_per_shaft_rev_estimate"), 1.0, "estimate");
            near(trueTpr, r.suggestions.get("flywheel.SHAFT_TICKS_PER_REV"), 1.0, "suggestion");
            check(String.join(" ", r.notes).contains("differs"), "flags the >5% mismatch");
            check(FlywheelConfig.SHAFT_TICKS_PER_REV == 28, "live config unchanged");
        });
    }
}
