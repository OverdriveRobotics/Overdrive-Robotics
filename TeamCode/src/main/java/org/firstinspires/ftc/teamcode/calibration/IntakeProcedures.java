package org.firstinspires.ftc.teamcode.calibration;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.commands.RobotConfig;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.tuning.Metrics;
import org.firstinspires.ftc.teamcode.tuning.RunLog;

/** Intake/transfer: direction, power, timing and transfer reliability, through the production {@code RunIntake} command. */
public final class IntakeProcedures {
    private IntakeProcedures() {}

    /** Runs one timed intake command and logs motor current/velocity. Returns false if aborted. */
    private static boolean runOnce(CalContext c, RunLog log, double t0, boolean reverse, double power, double ms) {
        RobotHardware r = c.robot();
        Command cmd = RobotCommands.runIntake(r, reverse, power, ms, null);
        Scheduler.schedule(cmd);
        try {
            double end = c.timeSec() + ms / 1000.0 + 0.5;
            c.tick();
            while (!c.abortRequested() && Scheduler.isScheduled(cmd) && c.timeSec() < end) {
                c.tick();
                log.add(c.timeSec() - t0, r.intakeAndTransferMotor.getPower(), r.intakeAndTransferMotor.getCurrent(CurrentUnit.AMPS),
                        r.intakeAndTransferMotor.getVelocity(), reverse ? -1 : 1);
            }
        } finally {
            Scheduler.cancel(cmd);   // end() stops the motor on every path
            c.tick();
        }
        return !c.abortRequested();
    }

    public static Procedure directionAndTransfer(final int cycles, final double runMs) {
        return new Procedure() {
            @Override public String name() { return "intake-transfer"; }
            @Override public String[] paramKeys() { return new String[]{"robot.INTAKE_POWER", "robot.FEED_POWER", "robot.INTAKE_JAM_AMPS", "robot.INTAKE_JAM_MS", "robot.DETECT_INTAKE_JAM"}; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.FEED);
                if (!(cycles >= 1 && cycles <= 20)) pre.add("cycles must be 1..20");
                if (!(runMs >= 200 && runMs <= 5000)) pre.add("run time must be 200..5000 ms");
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                ProcedureResult out = new ProcedureResult();
                RunLog log = new RunLog("t", "power", "amps", "velocity", "direction");
                out.log = log;
                double p = Math.min(RobotConfig.INTAKE_POWER, CalConfig.INTAKE_MAX_POWER);
                if (p < RobotConfig.INTAKE_POWER) out.notes.add("calibration power capped at " + p + " (config " + RobotConfig.INTAKE_POWER + ")");
                if (!c.confirm("Intake will run at " + p + ". Clear of fingers? A=go")) { out.aborted = true; return out; }
                double t0 = c.timeSec();
                if (!runOnce(c, log, t0, false, p, runMs)) { out.aborted = true; return out; }
                boolean fwdOk = c.confirm("Did the intake pull balls IN? A=yes B=no");
                if (!runOnce(c, log, t0, true, p, runMs)) { out.aborted = true; return out; }
                boolean revOk = c.confirm("Did the REVERSE push balls OUT? A=yes B=no");
                out.metrics.put("direction_ok", fwdOk && revOk ? 1.0 : 0.0);
                if (!fwdOk || !revOk) out.notes.add("direction wrong: change intakeAndTransferMotor.setDirection in RobotHardware (a wiring/mechanical fact, not a tunable)");
                int ok = 0, done = 0;
                for (int i = 0; i < cycles && !c.abortRequested(); i++) {
                    if (!c.confirm("Cycle " + (i + 1) + "/" + cycles + ": load a ball, A to run the transfer")) break;
                    if (!runOnce(c, log, t0, false, Math.min(RobotConfig.FEED_POWER, CalConfig.INTAKE_MAX_POWER), runMs)) { out.aborted = true; break; }
                    done++;
                    if (c.confirm("Did the ball transfer correctly? A=yes B=no")) ok++;
                }
                if (done > 0) out.metrics.put("transfer_success_rate", (double) ok / done);
                out.metrics.put("cycles_run", (double) done);
                if (log.size() > 5) {
                    double[] a = log.column("amps"), v = log.column("velocity");
                    out.metrics.put("current_mean_a", Metrics.mean(a));
                    out.metrics.put("current_peak_a", Metrics.max(a));
                    out.metrics.put("velocity_mean_abs", Metrics.mean(abs(v)));
                    if (Metrics.max(a) > 0.5) {
                        double sug = 1.5 * Metrics.max(a);
                        out.suggestions.put("robot.INTAKE_JAM_AMPS", Math.min(30, Math.round(sug * 2) / 2.0));
                        out.notes.add("jam threshold suggestion = 1.5x the highest normal current seen; jam detection stays OFF unless robot.DETECT_INTAKE_JAM is enabled");
                    } else out.notes.add("no current reading above 0.5 A: current sensing may be unavailable, jam threshold not suggested");
                }
                out.aborted |= c.abortRequested();
                return out;
            }
        };
    }

    private static double[] abs(double[] a) { double[] b = new double[a.length]; for (int i = 0; i < a.length; i++) b[i] = Math.abs(a[i]); return b; }
}
