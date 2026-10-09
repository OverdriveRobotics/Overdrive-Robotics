package org.firstinspires.ftc.teamcode.tuning;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.TreeMap;

/**
 * Result of one calibration procedure run, traceable to the exact parameters used (revision + hash of the effective
 * parameter set, plus the values themselves). {@code mode} is SIMULATION or HARDWARE and is never omitted.
 */
public final class RunReport {
    public String procedure, mode, note = "";
    public int baseRevision;
    public boolean usedCandidate;
    public String configHash = "";
    public long timestamp;
    public boolean aborted;
    public final Map<String, Double> metrics = new TreeMap<>();
    public final Map<String, Double> params = new TreeMap<>();
    public final List<String> notes = new ArrayList<>();

    public RunReport(String procedure, String mode) { this.procedure = procedure; this.mode = mode; }

    public File save(File runsDir) throws IOException {
        runsDir.mkdirs();
        int seq = 1;
        while (new File(runsDir, String.format("%s-%03d.report.properties", procedure, seq)).exists()) seq++;
        File f = new File(runsDir, String.format("%s-%03d.report.properties", procedure, seq));
        Properties p = new Properties();
        p.setProperty("procedure", procedure); p.setProperty("mode", mode); p.setProperty("note", note);
        p.setProperty("baseRevision", Integer.toString(baseRevision)); p.setProperty("usedCandidate", Boolean.toString(usedCandidate));
        p.setProperty("configHash", configHash); p.setProperty("timestamp", Long.toString(timestamp));
        p.setProperty("aborted", Boolean.toString(aborted));
        for (Map.Entry<String, Double> e : metrics.entrySet()) p.setProperty("m." + e.getKey(), e.getValue().toString());
        for (Map.Entry<String, Double> e : params.entrySet()) p.setProperty("p." + e.getKey(), e.getValue().toString());
        for (int i = 0; i < notes.size(); i++) p.setProperty("n." + String.format("%03d", i), notes.get(i));
        try (java.io.OutputStream o = Files.newOutputStream(f.toPath())) { p.store(o, "calibration run report"); }
        return f;
    }

    public static RunReport load(File f) throws IOException {
        Properties p = new Properties();
        try (java.io.Reader r = new java.io.InputStreamReader(new java.io.FileInputStream(f), StandardCharsets.UTF_8)) { p.load(r); }
        RunReport r = new RunReport(p.getProperty("procedure"), p.getProperty("mode"));
        r.note = p.getProperty("note", ""); r.baseRevision = Integer.parseInt(p.getProperty("baseRevision", "0"));
        r.usedCandidate = Boolean.parseBoolean(p.getProperty("usedCandidate")); r.configHash = p.getProperty("configHash", "");
        r.timestamp = Long.parseLong(p.getProperty("timestamp", "0")); r.aborted = Boolean.parseBoolean(p.getProperty("aborted"));
        java.util.TreeSet<String> names = new java.util.TreeSet<>(p.stringPropertyNames());
        for (String n : names) {
            if (n.startsWith("m.")) r.metrics.put(n.substring(2), Double.parseDouble(p.getProperty(n)));
            else if (n.startsWith("p.")) r.params.put(n.substring(2), Double.parseDouble(p.getProperty(n)));
            else if (n.startsWith("n.")) r.notes.add(p.getProperty(n));
        }
        return r;
    }

    /** Side-by-side comparison, flagging better/worse using {@link Metrics#HIGHER_IS_BETTER}. */
    public static List<String> compare(RunReport base, RunReport cur) {
        List<String> out = new ArrayList<>();
        out.add("baseline: " + base.procedure + " [" + base.mode + "] rev " + base.baseRevision + (base.usedCandidate ? "+candidate" : "") + " cfg " + base.configHash);
        out.add("current : " + cur.procedure + " [" + cur.mode + "] rev " + cur.baseRevision + (cur.usedCandidate ? "+candidate" : "") + " cfg " + cur.configHash);
        if (!base.mode.equals(cur.mode)) out.add("WARNING: comparing runs of different modes (" + base.mode + " vs " + cur.mode + ")");
        if (!base.procedure.equals(cur.procedure)) out.add("WARNING: different procedures");
        for (String k : cur.params.keySet()) {
            Double b = base.params.get(k);
            if (b != null && !b.equals(cur.params.get(k)) && !(b.isNaN() && cur.params.get(k).isNaN()))
                out.add(String.format("  param %-34s %s -> %s", k, b, cur.params.get(k)));
        }
        for (Map.Entry<String, Double> e : cur.metrics.entrySet()) {
            Double b = base.metrics.get(e.getKey());
            double c = e.getValue();
            String verdict = "";
            if (b != null && !Double.isNaN(b) && !Double.isNaN(c)) {
                boolean signedError = e.getKey().endsWith("_error");   // judge the size of an error, not its sign
                double bb = signedError ? Math.abs(b) : b, cc = signedError ? Math.abs(c) : c;
                double d = cc - bb, tol = 1e-9 + 0.02 * Math.abs(bb) + (signedError ? 1e-2 : 0);
                boolean higher = Metrics.HIGHER_IS_BETTER.contains(e.getKey());
                verdict = Math.abs(d) <= tol ? "same" : ((d > 0) == higher ? "better" : "WORSE");
            }
            out.add(String.format("  %-28s %12s -> %12s  %s", e.getKey(), b == null ? "n/a" : fmt(b), fmt(c), verdict));
        }
        return out;
    }

    private static String fmt(double v) { return Double.isNaN(v) ? "NaN" : String.format("%.4g", v); }
}
