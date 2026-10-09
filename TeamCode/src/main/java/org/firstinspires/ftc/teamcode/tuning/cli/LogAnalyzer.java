package org.firstinspires.ftc.teamcode.tuning.cli;

import org.firstinspires.ftc.teamcode.tuning.Metrics;
import org.firstinspires.ftc.teamcode.tuning.RunLog;
import org.firstinspires.ftc.teamcode.tuning.SysId;

import java.io.File;
import java.util.Map;

/**
 * Analyses CSV logs pulled from the robot (overdrive/runs/*.csv).
 *   LogAnalyzer sysid LOG.csv            needs columns t,volts,omega      -> kS/kV/kA fit
 *   LogAnalyzer step LOG.csv TARGET BAND needs columns t,measured|angle   -> step metrics
 *   LogAnalyzer show LOG.csv             header + metadata + row count
 */
public final class LogAnalyzer {
    private LogAnalyzer() {}

    public static void main(String[] a) throws Exception { System.exit(run(a, System.out)); }

    public static int run(String[] a, java.io.PrintStream out) throws Exception {
        if (a.length < 2) { out.println("usage: LogAnalyzer (sysid LOG | step LOG TARGET BAND | show LOG)"); return 2; }
        RunLog log = RunLog.load(new File(a[1]));
        out.println("log " + a[1] + ": " + log.size() + " rows, columns " + String.join(",", log.columns));
        for (Map.Entry<String, String> e : log.meta.entrySet()) out.println("  # " + e.getKey() + " = " + e.getValue());
        switch (a[0]) {
            case "show": return 0;
            case "sysid": {
                SysId.Fit f = SysId.fitVelocity(log.column("t"), log.column("volts"), log.column("omega"));
                if (!f.ok) { out.println("fit failed: " + f.problem); return 1; }
                out.println(String.format("a=%.5g  b=%.5g  kS=%.4g V  kV=%.6g  kA=%.6g  rmse=%.4g  r2=%.4f  samples=%d%s", f.a, f.b, f.kS, f.kV, f.kA, f.rmse, f.r2, f.samples,
                        f.kSIdentified ? "" : "  (single voltage level: kS not identifiable)"));
                out.println("suggested: flywheel.KS_VOLTS=" + f.kS + " flywheel.KV_VOLTS_PER_TICK_S=" + f.kV + " flywheel.KA_VOLTS_PER_TICK_S2=" + f.kA + " flywheel.USE_SYSID_MODEL=1");
                out.println("(apply with ConfigTool set ..., then validate with the step-response procedure)");
                return 0;
            }
            case "step": {
                String col = java.util.Arrays.asList(log.columns).contains("measured") ? "measured" : "angle";
                Map<String, Double> m = Metrics.step(log.column("t"), log.column(col), Double.parseDouble(a[2]), Double.parseDouble(a[3]), 0.25);
                for (Map.Entry<String, Double> e : m.entrySet()) out.println(String.format("  %-20s %s", e.getKey(), e.getValue()));
                return 0;
            }
            default: out.println("unknown analysis " + a[0]); return 2;
        }
    }
}
