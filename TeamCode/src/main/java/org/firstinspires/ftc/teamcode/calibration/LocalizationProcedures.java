package org.firstinspires.ftc.teamcode.calibration;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.control.AngleUtil;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.tuning.Metrics;
import org.firstinspires.ftc.teamcode.tuning.RunLog;
import org.firstinspires.ftc.teamcode.tuning.VelocityFrame;

/**
 * Localization and coordinate-frame checks. All of them are hand-driven: the robot is moved by the operator, the
 * drivetrain is never powered, and the results are compared against tape-measure ground truth.
 */
public final class LocalizationProcedures {
    private LocalizationProcedures() {}

    /** Place the robot at known poses and compare Pedro's pose to the truth. */
    public static Procedure referencePoses(final double[][] expected) {
        return new Procedure() {
            @Override public String name() { return "loc-reference"; }
            @Override public String[] paramKeys() { return new String[0]; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.LOCALIZATION);
                if (expected.length == 0) pre.add("no reference poses given");
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                ProcedureResult out = new ProcedureResult();
                RunLog log = new RunLog("t", "ref", "exp_x", "exp_y", "exp_h_deg", "x", "y", "h_deg", "pos_err", "head_err_deg");
                out.log = log;
                double[] pe = new double[expected.length], he = new double[expected.length];
                int n = 0;
                for (int i = 0; i < expected.length && !c.abortRequested(); i++) {
                    double[] e = expected[i];
                    if (!c.confirm(String.format("Place the robot at reference %d: x=%.1f y=%.1f heading=%.1f deg, hold still, A", i + 1, e[0], e[1], e[2]))) { out.aborted = true; break; }
                    c.tick();
                    Pose p = r.follower.pose();
                    double pos = Math.hypot(p.x() - e[0], p.y() - e[1]);
                    double he1 = Math.toDegrees(AngleUtil.wrap(p.heading() - Math.toRadians(e[2])));
                    pe[n] = pos; he[n] = Math.abs(he1); n++;
                    log.add(c.timeSec(), i, e[0], e[1], e[2], p.x(), p.y(), Math.toDegrees(p.heading()), pos, he1);
                }
                if (n > 0) {
                    double[] a = java.util.Arrays.copyOf(pe, n), b = java.util.Arrays.copyOf(he, n);
                    out.metrics.put("pos_error_mean_in", Metrics.mean(a)); out.metrics.put("pos_error_max_in", Metrics.max(a));
                    out.metrics.put("heading_error_max_deg", Metrics.max(b));
                }
                out.notes.add("errors are against YOUR tape-measured reference, so they include placement error; pod/IMU scale problems are fixed in the Pinpoint/Pedro configuration, not here");
                out.aborted |= c.abortRequested();
                return out;
            }
        };
    }

    /** Turn the robot one full revolution by hand: accumulated heading must be +-360 degrees. */
    public static Procedure headingRevolution() {
        return new Procedure() {
            @Override public String name() { return "loc-heading"; }
            @Override public String[] paramKeys() { return new String[0]; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.LOCALIZATION);
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                ProcedureResult out = new ProcedureResult();
                RunLog log = new RunLog("t", "heading_deg", "accum_deg");
                out.log = log;
                if (!c.confirm("Mark the robot's start orientation. Rotate it ONE full turn by hand, slowly, back onto the mark. A to start recording")) { out.aborted = true; return out; }
                double last = r.follower.pose().heading(), accum = 0, t0 = c.timeSec();
                while (!c.abortRequested()) {
                    c.tick();
                    double h = r.follower.pose().heading();
                    accum += AngleUtil.wrap(h - last);
                    last = h;
                    log.add(c.timeSec() - t0, Math.toDegrees(h), Math.toDegrees(accum));
                    if (c.timeSec() - t0 > 0.5 && c.confirm("Back on the mark? A=finish recording, B=keep going")) break;
                }
                double total = Math.toDegrees(Math.abs(accum));
                out.metrics.put("accumulated_deg", total);
                out.metrics.put("revolution_error_deg", total - 360.0);
                out.metrics.put("revolution_error_pct", (total - 360.0) / 360.0 * 100);
                out.aborted |= c.abortRequested();
                return out;
            }
        };
    }

    /** Push the robot a tape-measured distance in a straight line. */
    public static Procedure straightDistance() {
        return new Procedure() {
            @Override public String name() { return "loc-distance"; }
            @Override public String[] paramKeys() { return new String[0]; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.LOCALIZATION);
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                ProcedureResult out = new ProcedureResult();
                double truth = c.promptValue("Tape-measured distance you will push the robot (in)", 48, 1, 6, 200);
                if (!c.confirm("Place the robot at the start, then A")) { out.aborted = true; return out; }
                c.tick();
                Pose a = r.follower.pose();
                if (!c.confirm("Push it straight the measured distance, then A")) { out.aborted = true; return out; }
                c.tick();
                Pose b = r.follower.pose();
                double d = Math.hypot(b.x() - a.x(), b.y() - a.y());
                out.metrics.put("measured_in", truth);
                out.metrics.put("reported_in", d);
                out.metrics.put("scale_error_pct", (d - truth) / truth * 100);
                out.notes.add("A consistent scale error means the odometry pod calibration (Pinpoint/Pedro tuner), not a value stored here, needs correcting");
                return out;
            }
        };
    }

    /** Decide whether Follower.velocity() is field-frame by pushing the robot around at a non-zero heading. */
    public static Procedure velocityFrame(final double seconds) {
        return new Procedure() {
            @Override public String name() { return "loc-velframe"; }
            @Override public String[] paramKeys() { return new String[]{"turret.ROBOT_VELOCITY_IS_FIELD_FRAME"}; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.LOCALIZATION);
                if (!(seconds >= 3 && seconds <= 60)) pre.add("duration must be 3..60 s");
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                ProcedureResult out = new ProcedureResult();
                RunLog log = new RunLog("t", "heading_deg", "vx", "vy", "dx_dt", "dy_dt");
                out.log = log;
                if (!c.confirm("Turn the robot to roughly 90 deg heading, then PUSH it around by hand for " + (int) seconds + " s (forward, sideways, diagonal). A=start")) { out.aborted = true; return out; }
                VelocityFrame vf = new VelocityFrame();
                Pose prev = r.follower.pose();
                double prevT = c.timeSec(), t0 = prevT;
                while (!c.abortRequested() && c.timeSec() - t0 < seconds) {
                    c.tick();
                    Pose p = r.follower.pose();
                    Velocity v = r.follower.velocity();
                    double now = c.timeSec(), dt = now - prevT;
                    if (dt >= 0.05) {
                        vf.add(p.heading(), v.vx, v.vy, p.x() - prev.x(), p.y() - prev.y(), dt);
                        log.add(now - t0, Math.toDegrees(p.heading()), v.vx, v.vy, (p.x() - prev.x()) / dt, (p.y() - prev.y()) / dt);
                        prev = p; prevT = now;
                    }
                }
                VelocityFrame.Verdict verdict = vf.verdict();
                out.metrics.put("samples", (double) vf.samples());
                out.metrics.put("verdict_field_1_robot_2_unknown_0", verdict == VelocityFrame.Verdict.FIELD ? 1.0 : verdict == VelocityFrame.Verdict.ROBOT ? 2.0 : 0.0);
                if (verdict == VelocityFrame.Verdict.FIELD) { out.suggestions.put("turret.ROBOT_VELOCITY_IS_FIELD_FRAME", 1.0); out.notes.add("velocity matches pose-derived field velocity"); }
                else if (verdict == VelocityFrame.Verdict.ROBOT) { out.suggestions.put("turret.ROBOT_VELOCITY_IS_FIELD_FRAME", 0.0); out.notes.add("velocity matches ROBOT-frame interpretation"); }
                else out.notes.add("undetermined: need more motion at a heading away from 0/180 deg, at > 3 in/s");
                out.aborted |= c.abortRequested();
                return out;
            }
        };
    }

    /** Stand the robot centre on each target point; the pose there becomes the proposed target coordinates. */
    public static Procedure measureTargets() {
        return new Procedure() {
            @Override public String name() { return "loc-targets"; }
            @Override public String[] paramKeys() { return new String[]{"target.A_X", "target.A_Y", "target.B_X", "target.B_Y"}; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.LOCALIZATION);
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                ProcedureResult out = new ProcedureResult();
                String[] names = {"A", "B"};
                for (int i = 0; i < 2 && !c.abortRequested(); i++) {
                    if (!c.confirm("Place the robot's CENTRE exactly over target point " + names[i] + " (or hold a marker at the pose origin over it), then A. B=skip")) continue;
                    c.tick();
                    Pose p = r.follower.pose();
                    out.metrics.put("target_" + names[i] + "_x", p.x());
                    out.metrics.put("target_" + names[i] + "_y", p.y());
                    out.suggestions.put("target." + names[i] + "_X", p.x());
                    out.suggestions.put("target." + names[i] + "_Y", p.y());
                }
                out.notes.add("pose must be localized to the SAME field origin the autonomous uses (set at init)");
                out.aborted |= c.abortRequested();
                return out;
            }
        };
    }
}
