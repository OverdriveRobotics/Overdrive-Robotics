package org.firstinspires.ftc.teamcode.calibration;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.commands.RobotConfig;
import org.firstinspires.ftc.teamcode.control.FlywheelConfig;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;
import org.firstinspires.ftc.teamcode.tuning.Metrics;
import org.firstinspires.ftc.teamcode.tuning.RunLog;
import org.firstinspires.ftc.teamcode.tuning.SysId;

import java.util.Map;

/**
 * Flywheel calibration. Closed-loop procedures drive the real production path (FlywheelRegulator through the Ivy
 * scheduler), so they measure exactly what the match code does. Open-loop procedures (sysid, scaling) take exclusive
 * control of the shooter motors AFTER stopping the regulator, and hand them back clean.
 */
public final class FlywheelProcedures {
    private FlywheelProcedures() {}

    private static final String[] COMMON = {"flywheel.MAX_VELOCITY_ERROR", "flywheel.MODEL_STD", "flywheel.MEASUREMENT_STD",
            "flywheel.MODEL_IDENTIFIED", "flywheel.USE_SYSID_MODEL", "flywheel.KS_VOLTS", "flywheel.KV_VOLTS_PER_TICK_S",
            "flywheel.KA_VOLTS_PER_TICK_S2", "robot.FLYWHEEL_TOLERANCE", "robot.FLYWHEEL_STABLE_MS", "robot.FLYWHEEL_SPINUP_TIMEOUT_MS"};

    // ------------------------------------------------------------------ helpers

    /** Stops the flywheel through the production command and waits (bounded) for it to spin down. */
    static void stopAndWait(CalContext c, double belowVel, double maxSec) {
        RobotHardware r = c.robot();
        Scheduler.schedule(RobotCommands.stopFlywheel(r));
        double t0 = c.timeSec();
        c.tick();
        while (!c.abortRequested() && c.timeSec() - t0 < maxSec) {
            double v = (r.shooterLeft.getVelocity() + r.shooterRight.getVelocity()) / 2;
            if (!(Math.abs(v) > belowVel)) break;
            c.tick();
        }
    }

    private static double avgVel(RobotHardware r) { return (r.shooterLeft.getVelocity() + r.shooterRight.getVelocity()) / 2; }

    private static boolean badTarget(double target) {
        return !Double.isFinite(target) || target <= 0 || target > RobotConfig.MAX_FLYWHEEL_VELOCITY;
    }

    // ------------------------------------------------------------------ step response

    public static Procedure stepResponse(final double target, final double holdSec) {
        return new Procedure() {
            @Override public String name() { return "flywheel-step"; }
            @Override public String[] paramKeys() { return COMMON; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.FLYWHEEL, Preflight.Need.BATTERY);
                if (badTarget(target)) pre.add("invalid target velocity " + target);
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                ProcedureResult out = new ProcedureResult();
                RunLog log = new RunLog("t", "target", "measured", "error", "power", "vbat", "ready", "stable_ms");
                out.log = log;
                try {
                    c.status("spinning down before the step...");
                    stopAndWait(c, 0.05 * target, 25);
                    Storage.flywheelFault = null;
                    RobotCommands.ensureFlywheelRegulator(r);
                    Command up = RobotCommands.spinUpFlywheel(r, target, RobotConfig.FLYWHEEL_TOLERANCE,
                            RobotConfig.FLYWHEEL_STABLE_MS, holdSec * 1000);
                    Scheduler.schedule(up);
                    double t0 = c.timeSec(), firstReady = Double.NaN;
                    while (!c.abortRequested() && c.timeSec() - t0 < holdSec) {
                        c.tick();
                        double t = c.timeSec() - t0;
                        if (Double.isNaN(firstReady) && Storage.flywheelReady) firstReady = t;
                        log.add(t, target, Storage.flywheelMeasuredVelocity, target - Storage.flywheelMeasuredVelocity,
                                Storage.flywheelPower, r.batteryVolts(), Storage.flywheelReady ? 1 : 0, Storage.flywheelStableForMs);
                    }
                    out.aborted = c.abortRequested();
                    if (log.size() > 10) {
                        double[] t = log.column("t"), y = log.column("measured"), p = log.column("power"), vb = log.column("vbat");
                        out.metrics.putAll(Metrics.step(t, y, target, RobotConfig.FLYWHEEL_TOLERANCE, 0.25));
                        out.metrics.put("readiness_delay_s", firstReady);
                        out.metrics.put("saturation_fraction", Metrics.fractionAtOrAbove(p, 0.999));
                        out.metrics.put("power_range_violations", (double) Metrics.countOutside(p, 0, 1));
                        out.metrics.put("battery_min_v", Metrics.min(vb));
                        out.metrics.put("battery_mean_v", Metrics.mean(vb));
                        out.metrics.put("peak_velocity", Metrics.max(y));
                        out.metrics.put("fault", Storage.flywheelFault == null ? 0.0 : 1.0);
                    }
                    if (Storage.flywheelFault != null) out.notes.add("flywheel fault: " + Storage.flywheelFault);
                    out.notes.add("control path: " + Storage.flywheelMode);
                } finally {
                    Scheduler.schedule(RobotCommands.stopFlywheel(r));
                    c.tick();
                }
                return out;
            }
        };
    }

    // ------------------------------------------------------------------ disturbance recovery

    public static Procedure disturbance(final double target, final int cutMs) {
        return new Procedure() {
            @Override public String name() { return "flywheel-disturbance"; }
            @Override public String[] paramKeys() { return COMMON; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.FLYWHEEL, Preflight.Need.BATTERY);
                if (badTarget(target)) pre.add("invalid target velocity " + target);
                if (!(cutMs > 0 && cutMs <= 1000)) pre.add("disturbance length must be 1..1000 ms");
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                ProcedureResult out = new ProcedureResult();
                RunLog log = new RunLog("t", "target", "measured", "power", "ready");
                out.log = log;
                try {
                    RobotCommands.ensureFlywheelRegulator(r);
                    Command up = RobotCommands.spinUpFlywheel(r, target, RobotConfig.FLYWHEEL_TOLERANCE,
                            RobotConfig.FLYWHEEL_STABLE_MS, 15000);
                    Scheduler.schedule(up);
                    double t0 = c.timeSec();
                    while (!c.abortRequested() && Scheduler.isScheduled(up) && c.timeSec() - t0 < 16) c.tick();
                    if (!Storage.flywheelReady) { out.notes.add("never reached the target; no disturbance applied"); out.aborted = true; return out; }
                    double hold0 = c.timeSec();
                    while (!c.abortRequested() && c.timeSec() - hold0 < 1.0) {
                        c.tick();
                        log.add(c.timeSec() - t0, target, Storage.flywheelMeasuredVelocity, Storage.flywheelPower, Storage.flywheelReady ? 1 : 0);
                    }
                    double tDist = c.timeSec() - t0;
                    Storage.flywheelDisturbanceMs = cutMs;   // controlled disturbance: regulator cuts power for cutMs
                    double tNotReady = Double.NaN, tReady = Double.NaN;
                    double end = c.timeSec() + 8;
                    while (!c.abortRequested() && c.timeSec() < end) {
                        c.tick();
                        double t = c.timeSec() - t0;
                        log.add(t, target, Storage.flywheelMeasuredVelocity, Storage.flywheelPower, Storage.flywheelReady ? 1 : 0);
                        if (Double.isNaN(tNotReady) && !Storage.flywheelReady) tNotReady = t - tDist;
                        if (!Double.isNaN(tNotReady) && Double.isNaN(tReady) && Storage.flywheelReady) { tReady = t - tDist; break; }
                    }
                    out.aborted = c.abortRequested();
                    double[] t = log.column("t"), y = log.column("measured");
                    out.metrics.putAll(Metrics.recovery(t, y, tDist, target, RobotConfig.FLYWHEEL_TOLERANCE));
                    out.metrics.put("ready_lost_after_s", tNotReady);
                    out.metrics.put("ready_again_after_s", tReady);
                    out.metrics.put("fault", Storage.flywheelFault == null ? 0.0 : 1.0);
                    out.notes.add("disturbance = regulator power cut for " + cutMs + " ms (not a real ball)");
                } finally {
                    Storage.flywheelDisturbanceMs = 0;
                    Scheduler.schedule(RobotCommands.stopFlywheel(r));
                    c.tick();
                }
                return out;
            }
        };
    }

    // ------------------------------------------------------------------ open-loop sysid

    public static Procedure sysId(final double[] powers, final double holdSec, final double coastSec) {
        return new Procedure() {
            @Override public String name() { return "flywheel-sysid"; }
            @Override public String[] paramKeys() { return new String[]{"flywheel.KS_VOLTS", "flywheel.KV_VOLTS_PER_TICK_S", "flywheel.KA_VOLTS_PER_TICK_S2", "flywheel.USE_SYSID_MODEL"}; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.FLYWHEEL, Preflight.Need.BATTERY);
                for (double p : powers) if (!(p > 0 && p <= CalConfig.FLYWHEEL_MAX_POWER)) pre.add("power " + p + " outside (0, " + CalConfig.FLYWHEEL_MAX_POWER + "]");
                if (!(holdSec >= 1 && holdSec <= 10) || !(coastSec >= 1 && coastSec <= 20)) pre.add("hold must be 1..10 s and coast 1..20 s");
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                ProcedureResult out = new ProcedureResult();
                RunLog log = new RunLog("t", "volts", "omega", "power", "vbat");
                out.log = log;
                if (!c.confirm("Open-loop flywheel run up to " + (int) (100 * max(powers)) + "% power. Guard fitted, area clear? A=start")) {
                    out.aborted = true; out.notes.add("operator declined"); return out;
                }
                try {
                    stopAndWait(c, 20, 25);   // regulator off, wheel (nearly) stopped
                    r.shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    r.shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    double t0 = c.timeSec();
                    for (double p : powers) {
                        double segEnd = c.timeSec() + holdSec;
                        while (!c.abortRequested() && c.timeSec() < segEnd) {
                            double vb = r.batteryVolts();
                            if (!Double.isFinite(vb) || vb < CalConfig.MIN_BATTERY_V - 0.5) { out.notes.add("battery sagged to " + vb + " V: stopped"); out.aborted = true; break; }
                            r.shooterLeft.setPower(p); r.shooterRight.setPower(p);
                            c.tick();
                            log.add(c.timeSec() - t0, p * vb, avgVel(r), p, vb);
                        }
                        if (out.aborted || c.abortRequested()) break;
                    }
                    r.shooterLeft.setPower(0); r.shooterRight.setPower(0);
                    double coastEnd = c.timeSec() + coastSec;
                    while (!c.abortRequested() && c.timeSec() < coastEnd) {
                        c.tick();
                        log.add(c.timeSec() - t0, 0, avgVel(r), 0, r.batteryVolts());
                    }
                    out.aborted |= c.abortRequested();
                    if (!out.aborted) {
                        SysId.Fit f = SysId.fitVelocity(log.column("t"), log.column("volts"), log.column("omega"));
                        if (!f.ok) out.notes.add("fit failed: " + f.problem);
                        else {
                            out.metrics.put("fit_a", f.a); out.metrics.put("fit_b", f.b);
                            out.metrics.put("fit_kS_volts", f.kS); out.metrics.put("fit_kV", f.kV); out.metrics.put("fit_kA", f.kA);
                            out.metrics.put("fit_rmse", f.rmse); out.metrics.put("fit_r2", f.r2);
                            out.metrics.put("peak_velocity", Metrics.max(log.column("omega")));
                            out.suggestions.put("flywheel.KS_VOLTS", f.kS);
                            out.suggestions.put("flywheel.KV_VOLTS_PER_TICK_S", f.kV);
                            out.suggestions.put("flywheel.KA_VOLTS_PER_TICK_S2", f.kA);
                            out.suggestions.put("flywheel.USE_SYSID_MODEL", 1.0);
                            if (f.r2 < 0.8) out.notes.add("low R^2 (" + f.r2 + "): do not trust this fit; check logging and repeat");
                            out.notes.add("model identified from this run's data only; validate with the step-response and disturbance procedures before setting flywheel.MODEL_IDENTIFIED");
                        }
                    }
                } finally {
                    r.shooterLeft.setPower(0); r.shooterRight.setPower(0);
                    r.shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    r.shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Storage.flywheelPower = 0;
                }
                return out;
            }
        };
    }

    private static double max(double[] a) { double m = 0; for (double x : a) m = Math.max(m, x); return m; }

    // ------------------------------------------------------------------ encoder scaling

    /** Runs open loop at {@code power}, asks for a tachometer reading and checks ticks-per-rev. */
    public static Procedure scaling(final double power) {
        return new Procedure() {
            @Override public String name() { return "flywheel-scaling"; }
            @Override public String[] paramKeys() { return new String[]{"flywheel.SHAFT_TICKS_PER_REV", "flywheel.SHAFT_REVS_PER_FLYWHEEL_REV"}; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.FLYWHEEL, Preflight.Need.BATTERY);
                if (!(power > 0 && power <= CalConfig.FLYWHEEL_MAX_POWER)) pre.add("power outside (0, " + CalConfig.FLYWHEEL_MAX_POWER + "]");
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                ProcedureResult out = new ProcedureResult();
                RunLog log = new RunLog("t", "omega", "power");
                out.log = log;
                if (!c.confirm("Spin the flywheel at " + (int) (power * 100) + "% and read RPM with a tachometer. A=start")) { out.aborted = true; return out; }
                double rpm = Double.NaN;
                try {
                    stopAndWait(c, 20, 25);
                    r.shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    r.shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    double t0 = c.timeSec();
                    while (!c.abortRequested() && c.timeSec() - t0 < 4) {
                        r.shooterLeft.setPower(power); r.shooterRight.setPower(power);
                        c.tick();
                        log.add(c.timeSec() - t0, avgVel(r), power);
                    }
                    if (!c.abortRequested()) rpm = c.promptValue("Tachometer FLYWHEEL rpm (wheel still spinning)", 0, 50, 0, 100000);
                } finally {
                    r.shooterLeft.setPower(0); r.shooterRight.setPower(0);
                    r.shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    r.shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                out.aborted = c.abortRequested();
                if (log.size() > 20) {
                    double[] t = log.column("t"), w = log.column("omega");
                    double sum = 0; int n = 0;
                    for (int i = 0; i < t.length; i++) if (t[i] > 2.5) { sum += w[i]; n++; }
                    double mean = n == 0 ? Double.NaN : sum / n;
                    out.metrics.put("mean_ticks_per_s", mean);
                    out.metrics.put("measured_rpm", rpm);
                    if (rpm > 1 && mean > 0) {
                        double shaftRps = rpm / 60.0 * FlywheelConfig.SHAFT_REVS_PER_FLYWHEEL_REV;
                        double tpr = mean / shaftRps;
                        out.metrics.put("ticks_per_shaft_rev_estimate", tpr);
                        out.metrics.put("ticks_per_rev_error_pct", 100 * (FlywheelConfig.SHAFT_TICKS_PER_REV - tpr) / tpr);
                        out.suggestions.put("flywheel.SHAFT_TICKS_PER_REV", tpr);
                        if (Math.abs(tpr / FlywheelConfig.SHAFT_TICKS_PER_REV - 1) > 0.05)
                            out.notes.add("configured ticks/rev differs from the tachometer-derived value by >5%: check the encoder resolution and flywheel.SHAFT_REVS_PER_FLYWHEEL_REV");
                    } else out.notes.add("no tachometer value entered: scaling not checked");
                }
                return out;
            }
        };
    }

    /** Unused import guard. */
    @SuppressWarnings("unused") private static void unused(Map<String, Double> m) {}
}
