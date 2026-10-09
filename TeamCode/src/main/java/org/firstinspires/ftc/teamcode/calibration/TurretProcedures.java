package org.firstinspires.ftc.teamcode.calibration;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.control.AngleUtil;
import org.firstinspires.ftc.teamcode.control.TurretConfig;
import org.firstinspires.ftc.teamcode.control.TurretStateSpaceController;
import org.firstinspires.ftc.teamcode.control.TurretUtil;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;
import org.firstinspires.ftc.teamcode.tuning.Metrics;
import org.firstinspires.ftc.teamcode.tuning.RunLog;
import org.firstinspires.ftc.teamcode.tuning.SysId;

import java.util.ArrayList;
import java.util.List;

/**
 * Turret calibration. Every motorised step uses the production controller/conversions with a LOW voltage cap
 * ({@link CalConfig#TURRET_MAX_VOLTS}); manual steps leave the motor unpowered (coast) so it can be turned by hand.
 */
public final class TurretProcedures {
    private TurretProcedures() {}

    private static final String[] GEOM = {"turret.SHAFT_TICKS_PER_REV", "turret.SHAFT_REVS_PER_TURRET_REV", "turret.ENCODER_SIGN",
            "turret.START_ANGLE_RAD", "turret.MIN_ANGLE_RAD", "turret.MAX_ANGLE_RAD"};
    private static final String[] CTRL = {"turret.MAX_ANGLE_ERROR_RAD", "turret.MAX_VELOCITY_ERROR_RAD_S",
            "turret.MAX_REFERENCE_VELOCITY_RAD_S", "turret.VELOCITY_FILTER_ALPHA", "turret.INTEGRAL_GAIN",
            "turret.ALIGN_TOLERANCE_RAD", "turret.ALIGN_VELOCITY_TOL_RAD_S", "turret.LOAD_INERTIA_KG_M2", "turret.VISCOUS_FRICTION"};

    static double angle(RobotHardware r) { return TurretUtil.ticksToAngle(r.turret.getCurrentPosition() - Storage.turretZeroTicks); }
    static double velocity(RobotHardware r) { return TurretUtil.ticksPerSecToRadPerSec(r.turret.getVelocity()); }

    private static void coast(RobotHardware r) {
        r.turret.setPower(0);
    }

    private static boolean nearLimit(double a) {
        return !TurretConfig.CONTINUOUS && (a < TurretConfig.MIN_ANGLE_RAD + TurretConfig.LIMIT_MARGIN_RAD
                || a > TurretConfig.MAX_ANGLE_RAD - TurretConfig.LIMIT_MARGIN_RAD);
    }

    // ------------------------------------------------------------------ encoder direction

    public static Procedure encoderDirection() {
        return new Procedure() {
            @Override public String name() { return "turret-direction"; }
            @Override public String[] paramKeys() { return new String[]{"turret.ENCODER_SIGN"}; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.TURRET, Preflight.Need.BATTERY);
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                ProcedureResult out = new ProcedureResult();
                RunLog log = new RunLog("t", "ticks", "power");
                out.log = log;
                if (!c.confirm("Turret will nudge briefly with POSITIVE power. Centred, clear of cables and hard stops? A=go")) { out.aborted = true; return out; }
                int t0Ticks = r.turret.getCurrentPosition();
                try {
                    r.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    double power = Math.min(0.15, CalConfig.TURRET_MAX_VOLTS / Math.max(1, r.batteryVolts()));
                    double t0 = c.timeSec();
                    while (!c.abortRequested() && c.timeSec() - t0 < 0.3) {
                        r.turret.setPower(power);
                        c.tick();
                        log.add(c.timeSec() - t0, r.turret.getCurrentPosition() - t0Ticks, power);
                    }
                } finally { coast(r); }
                int delta = r.turret.getCurrentPosition() - t0Ticks;
                out.aborted = c.abortRequested();
                out.metrics.put("delta_ticks", (double) delta);
                if (!out.aborted) {
                    boolean ccw = c.confirm("Did the turret turn COUNTER-CLOCKWISE (seen from above)? A=yes B=no");
                    out.suggestions.putAll(TurretCalLogic.directionSuggestion(delta, ccw, out.notes));
                }
                return out;
            }
        };
    }

    // ------------------------------------------------------------------ ticks per turret revolution (hand rotation)

    public static Procedure ticksPerRev(final int repeats, final double knownAngleDeg) {
        return new Procedure() {
            @Override public String name() { return "turret-ticksperrev"; }
            @Override public String[] paramKeys() { return new String[]{"turret.SHAFT_TICKS_PER_REV", "turret.SHAFT_REVS_PER_TURRET_REV"}; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.TURRET);
                if (!(repeats >= 2 && repeats <= 10)) pre.add("repeats must be 2..10");
                if (!(knownAngleDeg >= 30 && knownAngleDeg <= 360)) pre.add("known angle must be 30..360 degrees");
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                ProcedureResult out = new ProcedureResult();
                RunLog log = new RunLog("t", "repeat", "ticks_delta");
                out.log = log;
                coast(r);   // motor unpowered: turn the turret by hand
                double[] deltas = new double[repeats];
                int got = 0;
                for (int i = 0; i < repeats && !c.abortRequested(); i++) {
                    if (!c.confirm("Repeat " + (i + 1) + "/" + repeats + ": put the turret on mark 0, then A")) { out.aborted = true; break; }
                    int a = r.turret.getCurrentPosition();
                    if (!c.confirm("Rotate it BY HAND exactly " + knownAngleDeg + " deg (same direction every time), then A")) { out.aborted = true; break; }
                    int b = r.turret.getCurrentPosition();
                    deltas[got++] = b - a;
                    log.add(c.timeSec(), i, b - a);
                }
                if (got >= 2) {
                    double[] d = java.util.Arrays.copyOf(deltas, got);
                    double[] ms = TurretCalLogic.ticksPerTurretRev(d, knownAngleDeg);
                    out.metrics.put("ticks_per_turret_rev", ms[0]);
                    out.metrics.put("ticks_per_turret_rev_stddev", ms[1]);
                    out.metrics.put("encoder_spread_pct", ms[0] > 0 ? 100 * ms[1] / ms[0] : Double.NaN);
                    double configured = TurretConfig.SHAFT_TICKS_PER_REV * TurretConfig.SHAFT_REVS_PER_TURRET_REV;
                    out.metrics.put("configured_ticks_per_turret_rev", configured);
                    out.metrics.put("configured_vs_measured_pct", 100 * (configured - ms[0]) / ms[0]);
                    double ratio = TurretCalLogic.externalRatio(ms[0], TurretConfig.SHAFT_TICKS_PER_REV);
                    out.suggestions.put("turret.SHAFT_REVS_PER_TURRET_REV", ratio);
                    out.notes.add("external ratio = measured ticks/turret-rev divided by turret.SHAFT_TICKS_PER_REV (" + TurretConfig.SHAFT_TICKS_PER_REV
                            + "). That value must be the encoder resolution of the SAME shaft the motor datasheet numbers describe, or the internal gearbox is counted twice.");
                    if (ms[1] / ms[0] > 0.02) out.notes.add("repeats disagree by more than 2%: hand-rotation or mark alignment is not repeatable enough");
                } else out.notes.add("need at least two completed repeats");
                out.aborted |= c.abortRequested();
                return out;
            }
        };
    }

    // ------------------------------------------------------------------ mechanical limits (hand-driven)

    public static Procedure limits() {
        return new Procedure() {
            @Override public String name() { return "turret-limits"; }
            @Override public String[] paramKeys() { return GEOM; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.TURRET);
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                ProcedureResult out = new ProcedureResult();
                coast(r);
                if (!c.confirm("Turret unpowered. Gently push it to the MINIMUM (clockwise) hard stop, then A")) { out.aborted = true; return out; }
                double lo = angle(r);
                if (!c.confirm("Now gently push it to the MAXIMUM (counter-clockwise) hard stop, then A")) { out.aborted = true; return out; }
                double hi = angle(r);
                if (hi < lo) { double t = lo; lo = hi; hi = t; out.notes.add("stops were entered in reverse order; swapped"); }
                out.metrics.put("measured_min_rad", lo);
                out.metrics.put("measured_max_rad", hi);
                out.metrics.put("range_deg", Math.toDegrees(hi - lo));
                out.metrics.put("start_angle_inside_range", TurretConfig.START_ANGLE_RAD >= lo && TurretConfig.START_ANGLE_RAD <= hi ? 1.0 : 0.0);
                if (hi - lo > 2 * CalConfig.LIMIT_INSET_RAD + 0.2) {
                    out.suggestions.put("turret.MIN_ANGLE_RAD", lo + CalConfig.LIMIT_INSET_RAD);
                    out.suggestions.put("turret.MAX_ANGLE_RAD", hi - CalConfig.LIMIT_INSET_RAD);
                    out.notes.add("proposed limits are the measured stops pulled in by " + Math.toDegrees(CalConfig.LIMIT_INSET_RAD) + " deg; the conversion used is the CURRENT one (calibrate ticks-per-rev first)");
                } else out.notes.add("measured range is implausibly small");
                return out;
            }
        };
    }

    // ------------------------------------------------------------------ zero / drift

    public static Procedure zeroCheck() {
        return new Procedure() {
            @Override public String name() { return "turret-zero"; }
            @Override public String[] paramKeys() { return new String[]{"turret.START_ANGLE_RAD"}; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.TURRET);
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                ProcedureResult out = new ProcedureResult();
                coast(r);
                if (!c.confirm("Place the turret on its robot-FORWARD mark, then A")) { out.aborted = true; return out; }
                double a = angle(r);
                double expectedDeg = c.promptValue("True angle of that mark (deg, 0 = forward, + = CCW)", 0, 1, -180, 180);
                out.metrics.put("angle_at_mark_deg", Math.toDegrees(a));
                out.metrics.put("zero_error_deg", Math.toDegrees(a) - expectedDeg);
                out.notes.add("The encoder is zeroed at OpMode init: centre the turret on this mark BEFORE pressing INIT. A large error here means init happened off the mark.");
                if (Math.abs(Math.toDegrees(a) - expectedDeg) > 1.0)
                    out.notes.add("turret.START_ANGLE_RAD only needs changing if the turret is NOT on the mark at init; otherwise re-centre and re-init");
                return out;
            }
        };
    }

    // ------------------------------------------------------------------ closed-loop step response

    public static Procedure stepResponse(final double stepDeg, final double holdSec) {
        return new Procedure() {
            @Override public String name() { return "turret-step"; }
            @Override public String[] paramKeys() { return CTRL; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.TURRET, Preflight.Need.BATTERY);
                double step = Math.toRadians(stepDeg);
                double a0 = Double.isNaN(step) ? 0 : angle(r);
                if (!(Math.abs(stepDeg) > 0 && Math.abs(stepDeg) <= 60)) pre.add("step must be within 0..60 deg");
                if (!(holdSec >= 0.5 && holdSec <= 8)) pre.add("hold must be 0.5..8 s");
                if (!TurretConfig.CONTINUOUS && (a0 + Math.abs(step) > TurretConfig.MAX_ANGLE_RAD - TurretConfig.LIMIT_MARGIN_RAD
                        || a0 - Math.abs(step) < TurretConfig.MIN_ANGLE_RAD + TurretConfig.LIMIT_MARGIN_RAD))
                    pre.add("the +-step would reach the turret limits from the current angle " + Math.toDegrees(a0) + " deg");
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                java.util.List<String> ph = Preflight.placeholdersInUse("turret.SHAFT_TICKS_PER_REV", "turret.SHAFT_REVS_PER_TURRET_REV",
                        "turret.LOAD_INERTIA_KG_M2", "turret.VISCOUS_FRICTION", "turret.MIN_ANGLE_RAD", "turret.MAX_ANGLE_RAD");
                if (!ph.isEmpty() && !c.confirm("PLACEHOLDER values in use: " + ph + ". Results are not meaningful and motion may be wrong. Continue at <=" + CalConfig.TURRET_MAX_VOLTS + " V? A=yes")) {
                    ProcedureResult x = new ProcedureResult(); x.aborted = true; x.notes.add("operator declined with placeholders in use"); return x;
                }
                ProcedureResult out = new ProcedureResult();
                RunLog log = new RunLog("t", "target", "angle", "error", "ticks", "velocity", "power", "volts", "saturated", "vbat");
                out.log = log;
                TurretStateSpaceController.Params p = TurretStateSpaceController.Params.fromConfig();
                p.maxVolts = Math.min(p.maxVolts, CalConfig.TURRET_MAX_VOLTS);
                TurretStateSpaceController ctl;
                try { ctl = new TurretStateSpaceController(p); }
                catch (RuntimeException e) { return ProcedureResult.refused(java.util.Collections.singletonList("controller cannot be built: " + e.getMessage())); }
                if (!c.confirm("Turret will move +-" + stepDeg + " deg at <=" + CalConfig.TURRET_MAX_VOLTS + " V. Clear? A=go, BACK=abort")) { out.aborted = true; return out; }
                try {
                    r.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    double[] targets = {a0 + step, a0 - step, a0};
                    double t0 = c.timeSec(), last = t0;
                    int violations = 0;
                    for (int s = 0; s < targets.length && !c.abortRequested(); s++) {
                        double end = c.timeSec() + holdSec;
                        while (!c.abortRequested() && c.timeSec() < end) {
                            double now = c.timeSec(), dt = Math.max(1e-3, now - last);
                            last = now;
                            double ang = angle(r), vel = velocity(r), vb = r.batteryVolts();
                            double pw = ctl.update(targets[s], 0, ang, vel, vb, dt);
                            if (!TurretConfig.CONTINUOUS) pw = TurretUtil.guardLimits(pw, ang, TurretConfig.MIN_ANGLE_RAD, TurretConfig.MAX_ANGLE_RAD, TurretConfig.LIMIT_MARGIN_RAD);
                            if (ctl.hasFault()) { out.notes.add("controller fault (battery/encoder invalid): stopped"); out.aborted = true; break; }
                            if (!TurretConfig.CONTINUOUS && (ang < TurretConfig.MIN_ANGLE_RAD - 0.05 || ang > TurretConfig.MAX_ANGLE_RAD + 0.05)) { violations++; out.notes.add("beyond mechanical limits: stopped"); out.aborted = true; break; }
                            r.turret.setPower(pw);
                            c.tick();
                            log.add(now - t0, targets[s], ang, targets[s] - ang, r.turret.getCurrentPosition(), vel, pw, ctl.lastVolts(), ctl.isSaturated() ? 1 : 0, vb);
                            if (c.abortRequested()) break;
                        }
                        if (out.aborted) break;
                    }
                    out.aborted |= c.abortRequested();
                    out.metrics.put("limit_violations", (double) violations);
                    if (log.size() > 20) {
                        double[] t = log.column("t"), tgt = log.column("target"), ang = log.column("angle"), pw = log.column("power");
                        RunLog first = log.window(0, holdSec);
                        out.metrics.putAll(Metrics.step(first.column("t"), first.column("angle"), a0 + step, Math.toRadians(1.0), 0.3));
                        out.metrics.putAll(Metrics.tracking(t, tgt, ang, 0.3));
                        out.metrics.put("saturation_fraction", Metrics.fractionAtOrAbove(log.column("saturated"), 1.0));
                        out.metrics.put("peak_power", Metrics.max(pw));
                        out.metrics.put("peak_velocity_rad_s", Math.max(Metrics.max(log.column("velocity")), -Metrics.min(log.column("velocity"))));
                    }
                } finally { coast(r); }
                return out;
            }
        };
    }

    // ------------------------------------------------------------------ pulse sysid

    public static Procedure sysId(final double volts) {
        return new Procedure() {
            @Override public String name() { return "turret-sysid"; }
            @Override public String[] paramKeys() { return new String[]{"turret.LOAD_INERTIA_KG_M2", "turret.VISCOUS_FRICTION"}; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.TURRET, Preflight.Need.BATTERY);
                if (!(volts > 0 && volts <= CalConfig.TURRET_MAX_VOLTS)) pre.add("volts must be within (0, " + CalConfig.TURRET_MAX_VOLTS + "]");
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                java.util.List<String> ph = Preflight.placeholdersInUse("turret.SHAFT_TICKS_PER_REV", "turret.SHAFT_REVS_PER_TURRET_REV");
                ProcedureResult out = new ProcedureResult();
                if (!ph.isEmpty()) { out.refused = true; out.notes.add("calibrate ticks-per-rev first (placeholders in use: " + ph + "); velocities would be mis-scaled"); return out; }
                RunLog log = new RunLog("t", "volts", "omega");
                out.log = log;
                if (!c.confirm("Turret will pulse +-" + volts + " V for ~0.4 s, alternating. Centred and clear? A=go")) { out.aborted = true; return out; }
                List<double[]> rows = new ArrayList<>();
                try {
                    r.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    double t0 = c.timeSec();
                    double[] levels = {0.5, 1.0, 0.75};   // several voltages so static friction and gain can be separated
                    for (int k = 0; k < 6 && !c.abortRequested(); k++) {
                        double sgn = (k % 2 == 0) ? 1 : -1, end = c.timeSec() + 0.4, pv = volts * levels[(k / 2) % 3];
                        while (!c.abortRequested() && c.timeSec() < end) {
                            double vb = r.batteryVolts(), ang = angle(r);
                            if (nearLimit(ang)) { out.notes.add("approached a limit: stopped"); out.aborted = true; break; }
                            r.turret.setPower(sgn * pv / vb);
                            c.tick();
                            double w = velocity(r);
                            log.add(c.timeSec() - t0, sgn * pv, w);
                            rows.add(new double[]{c.timeSec() - t0, pv, sgn * w});
                        }
                        r.turret.setPower(0);
                        double coastEnd = c.timeSec() + 0.35;
                        while (!c.abortRequested() && c.timeSec() < coastEnd) { c.tick(); log.add(c.timeSec() - t0, 0, velocity(r)); rows.add(new double[]{c.timeSec() - t0, 0, sgn * velocity(r)}); }
                        if (out.aborted) break;
                    }
                    out.aborted |= c.abortRequested();
                } finally { coast(r); }
                if (!out.aborted && rows.size() > 30) {
                    double[] t = new double[rows.size()], v = new double[rows.size()], w = new double[rows.size()];
                    for (int i = 0; i < t.length; i++) { t[i] = rows.get(i)[0]; v[i] = rows.get(i)[1]; w[i] = rows.get(i)[2]; }
                    SysId.Fit f = SysId.fitVelocity(t, v, w);
                    if (!f.ok) out.notes.add("fit failed: " + f.problem);
                    else {
                        out.metrics.put("fit_a", f.a); out.metrics.put("fit_b", f.b); out.metrics.put("fit_kS_volts", f.kS);
                        out.metrics.put("fit_rmse", f.rmse); out.metrics.put("fit_r2", f.r2);
                        double[] jb = SysId.turretInertiaFriction(f.a, f.b, TurretConfig.SHAFT_REVS_PER_TURRET_REV, TurretConfig.SHAFT_STALL_TORQUE_NM,
                                TurretConfig.STALL_CURRENT_A, TurretConfig.NOMINAL_VOLTAGE, TurretConfig.SHAFT_FREE_SPEED_RAD_S);
                        if (jb != null) {
                            out.suggestions.put("turret.LOAD_INERTIA_KG_M2", jb[0]);
                            out.suggestions.put("turret.VISCOUS_FRICTION", jb[1]);
                        }
                        if (f.kSIdentified && f.kS > 0) out.suggestions.put("turret.STATIC_FRICTION_VOLTS", f.kS);
                        out.notes.add("inertia/friction are back-computed from the fitted (a,b) AND the motor datasheet constants in the config; they are only as good as those constants");
                    }
                }
                return out;
            }
        };
    }

    // ------------------------------------------------------------------ production aiming accuracy

    /** Runs the production AimTurret. The operator may turn the robot by hand to exercise heading compensation. */
    public static Procedure aimAccuracy(final double seconds, final double switchAtSec) {
        return new Procedure() {
            @Override public String name() { return "turret-aim"; }
            @Override public String[] paramKeys() { return CTRL; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.TURRET, Preflight.Need.LOCALIZATION, Preflight.Need.BATTERY);
                if (!(seconds >= 2 && seconds <= 60)) pre.add("duration must be 2..60 s");
                Storage.FieldPoint t = Storage.activeTarget();
                if (t == null || !t.isValid()) pre.add("active target point is not set (set target.A_/B_ in the candidate)");
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                ProcedureResult out = new ProcedureResult();
                RunLog log = new RunLog("t", "x", "y", "heading", "desired", "angle", "error", "ticks", "velocity", "power", "saturated", "fault", "target_index");
                out.log = log;
                if (!c.confirm("Turret will aim at target " + (Storage.activeTargetIndex == 0 ? "A" : "B") + " (<=" + CalConfig.TURRET_MAX_VOLTS + " V). Rotate the ROBOT by hand to test heading compensation. A=go")) { out.aborted = true; return out; }
                boolean oldCfg = TurretConfig.HARDWARE_CONFIGURED;
                double oldVolts = TurretConfig.MAX_CONTROL_VOLTS;
                Command aim = RobotCommands.aimTurret(r);
                try {
                    TurretConfig.HARDWARE_CONFIGURED = true;   // temporary, restored below; capped voltage
                    TurretConfig.MAX_CONTROL_VOLTS = Math.min(oldVolts, CalConfig.TURRET_MAX_VOLTS);
                    Scheduler.schedule(aim);
                    double t0 = c.timeSec();
                    boolean switched = false;
                    double switchT = Double.NaN, realignT = Double.NaN;
                    int faults = 0, aligned = 0, n = 0;
                    while (!c.abortRequested() && c.timeSec() - t0 < seconds) {
                        c.tick();
                        double now = c.timeSec() - t0;
                        if (!switched && switchAtSec > 0 && now >= switchAtSec) {
                            Storage.toggleTarget(); switched = true; switchT = now;
                        }
                        Pose p = r.follower.pose();
                        boolean fault = Storage.turretFault != null;
                        if (fault) faults++;
                        double err = Storage.turretError;
                        boolean al = !fault && Double.isFinite(err) && Math.abs(err) <= TurretConfig.ALIGN_TOLERANCE_RAD;
                        if (now > 1.0) { n++; if (al) aligned++; }
                        if (switched && Double.isNaN(realignT) && al && now > switchT + 0.1) realignT = now - switchT;
                        log.add(now, p.x(), p.y(), p.heading(), Storage.turretDesiredAngle, Storage.turretAngle, err,
                                r.turret.getCurrentPosition(), Storage.turretVelocity, Storage.turretPower,
                                Storage.turretSaturated ? 1 : 0, fault ? 1 : 0, Storage.activeTargetIndex);
                    }
                    out.aborted = c.abortRequested();
                    double[] tt = log.column("t");
                    double[] zero = new double[tt.length], err = log.column("error");
                    for (int i = 0; i < err.length; i++) if (Double.isNaN(err[i])) err[i] = Double.NaN;
                    out.metrics.putAll(Metrics.tracking(tt, zero, negate(err), 1.0));
                    double rmsDeg = Math.toDegrees(out.metrics.get("tracking_rms")), maxDeg = Math.toDegrees(out.metrics.get("tracking_max"));
                    out.metrics.put("aim_rms_deg", rmsDeg);
                    out.metrics.put("aim_max_deg", maxDeg);
                    out.metrics.put("aligned_fraction", n == 0 ? Double.NaN : (double) aligned / n);
                    out.metrics.put("fault_samples", (double) faults);
                    out.metrics.put("saturation_fraction", Metrics.fractionAtOrAbove(log.column("saturated"), 1.0));
                    if (switched) out.metrics.put("realign_time_s", realignT);
                    out.metrics.remove("tracking_rms"); out.metrics.remove("tracking_max");
                    if (!out.aborted) {
                        double obs = c.promptValue("Observed aim offset of the real shot/laser (deg, + = left of target, 0 = on target)", 0, 0.5, -45, 45);
                        out.metrics.put("observed_aim_offset_deg", obs);
                    }
                } finally {
                    Scheduler.cancel(aim);
                    TurretConfig.HARDWARE_CONFIGURED = oldCfg;
                    TurretConfig.MAX_CONTROL_VOLTS = oldVolts;
                    r.turret.setPower(0);
                    c.tick();
                }
                return out;
            }
        };
    }

    private static double[] negate(double[] a) { double[] b = new double[a.length]; for (int i = 0; i < a.length; i++) b[i] = -a[i]; return b; }

    @SuppressWarnings("unused") private static double wrap(double a) { return AngleUtil.wrap(a); }
}
