package org.firstinspires.ftc.teamcode.calibration;

import org.firstinspires.ftc.teamcode.commands.RobotConfig;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.tuning.Metrics;
import org.firstinspires.ftc.teamcode.tuning.RunLog;

/**
 * Stopper servo. There is NO position feedback, so every timing here is the operator's observation (reaction time
 * included, which makes it conservative). A commanded position is never treated as proof of physical position.
 */
public final class StopperProcedures {
    private StopperProcedures() {}

    /** Moves the servo to {@code target} at a limited rate (never jumps), stopping early on abort. */
    private static void slewTo(CalContext c, double target) {
        RobotHardware r = c.robot();
        double pos = Double.isNaN(r.stopper.getPosition()) ? target : r.stopper.getPosition();
        double last = c.timeSec();
        while (!c.abortRequested() && Math.abs(target - pos) > 1e-6) {
            c.tick();
            double now = c.timeSec(), maxStep = CalConfig.STOPPER_JOG_RATE * Math.max(0.005, now - last);
            last = now;
            pos += Math.max(-maxStep, Math.min(maxStep, target - pos));
            r.stopper.setPosition(pos);
        }
    }

    /** Jog the endpoints by hand, slowly, and propose them. */
    public static Procedure endpoints() {
        return new Procedure() {
            @Override public String name() { return "stopper-endpoints"; }
            @Override public String[] paramKeys() { return new String[]{"robot.STOPPER_CLOSED", "robot.STOPPER_OPEN"}; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.STOPPER);
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                ProcedureResult out = new ProcedureResult();
                try {
                    if (!c.confirm("Stopper will move SLOWLY. Remove balls/fingers from the gate. A=go")) { out.aborted = true; return out; }
                    double closed = jog(c, "CLOSED position (blocks balls)", RobotConfig.STOPPER_CLOSED);
                    if (Double.isNaN(closed)) { out.aborted = true; return out; }
                    double open = jog(c, "OPEN position (lets balls through)", RobotConfig.STOPPER_OPEN);
                    if (Double.isNaN(open)) { out.aborted = true; return out; }
                    out.metrics.put("closed", closed);
                    out.metrics.put("open", open);
                    out.metrics.put("travel", Math.abs(open - closed));
                    if (Math.abs(open - closed) < 0.05) out.notes.add("open and closed are nearly identical: not a usable gate");
                    else { out.suggestions.put("robot.STOPPER_CLOSED", closed); out.suggestions.put("robot.STOPPER_OPEN", open); }
                } finally { safe(c); }
                out.aborted |= c.abortRequested();
                return out;
            }

            private double jog(CalContext c, String what, double start) {
                double v = start;
                for (int i = 0; i < 12 && !c.abortRequested(); i++) {
                    slewTo(c, v);
                    if (c.confirm(what + " = " + String.format("%.3f", v) + " looks right? A=keep, B=adjust")) return v;
                    v = c.promptValue("New " + what, v, 0.01, 0, 1);
                }
                return Double.NaN;
            }
        };
    }

    /** Operator-timed open/close delays. */
    public static Procedure timing(final int repeats) {
        return new Procedure() {
            @Override public String name() { return "stopper-timing"; }
            @Override public String[] paramKeys() { return new String[]{"robot.STOPPER_SETTLE_MS", "robot.STOPPER_CLOSE_SETTLE_MS", "robot.STOPPER_OPEN", "robot.STOPPER_CLOSED"}; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.STOPPER);
                if (!(repeats >= 2 && repeats <= 10)) pre.add("repeats must be 2..10");
                if (RobotConfig.STOPPER_OPEN == RobotConfig.STOPPER_CLOSED) pre.add("open and closed positions are identical");
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                ProcedureResult out = new ProcedureResult();
                RunLog log = new RunLog("t", "repeat", "open_ms", "close_ms");
                out.log = log;
                double[] open = new double[repeats], close = new double[repeats];
                int n = 0;
                try {
                    if (!c.confirm("Stopper will SNAP open/closed (as in a match). Clear the gate. A=go; during each move press A the instant it has FULLY settled")) { out.aborted = true; return out; }
                    slewTo(c, RobotConfig.STOPPER_CLOSED);
                    for (int i = 0; i < repeats && !c.abortRequested(); i++) {
                        double t = timedMove(c, RobotConfig.STOPPER_OPEN, "OPEN");
                        if (Double.isNaN(t)) { out.aborted = true; break; }
                        double t2 = timedMove(c, RobotConfig.STOPPER_CLOSED, "CLOSE");
                        if (Double.isNaN(t2)) { out.aborted = true; break; }
                        open[n] = t; close[n] = t2; n++;
                        log.add(c.timeSec(), i, t, t2);
                    }
                } finally { safe(c); }
                if (n >= 2) {
                    double[] o = java.util.Arrays.copyOf(open, n), cl = java.util.Arrays.copyOf(close, n);
                    out.metrics.put("open_mean_ms", Metrics.mean(o)); out.metrics.put("open_max_ms", Metrics.max(o)); out.metrics.put("open_stddev_ms", Metrics.stddev(o));
                    out.metrics.put("close_mean_ms", Metrics.mean(cl)); out.metrics.put("close_max_ms", Metrics.max(cl)); out.metrics.put("close_stddev_ms", Metrics.stddev(cl));
                    out.suggestions.put("robot.STOPPER_SETTLE_MS", Math.ceil(Metrics.max(o) * 1.2 / 10) * 10);
                    out.suggestions.put("robot.STOPPER_CLOSE_SETTLE_MS", Math.ceil(Metrics.max(cl) * 1.2 / 10) * 10);
                    out.notes.add("times are OPERATOR-observed (include human reaction ~150-250 ms) with a 20% margin; they are an upper-bound estimate, not a servo measurement");
                } else out.notes.add("need at least two completed repeats");
                out.aborted |= c.abortRequested();
                return out;
            }

            /** Commands the position and returns ms until the operator confirms it settled; NaN on abort. */
            private double timedMove(CalContext c, double pos, String what) {
                c.status("commanding " + what);
                double t0 = c.timeSec();
                c.robot().stopper.setPosition(pos);
                boolean ok = c.confirm(what + " settled? press A now");
                double ms = (c.timeSec() - t0) * 1000.0;
                return ok ? ms : Double.NaN;
            }
        };
    }

    static void safe(CalContext c) {
        RobotHardware r = c.robot();
        if (r.stopper != null) r.stopper.setPosition(RobotConfig.STOPPER_CLOSED);
    }
}
