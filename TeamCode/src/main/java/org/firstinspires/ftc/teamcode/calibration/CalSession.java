package org.firstinspires.ftc.teamcode.calibration;

import org.firstinspires.ftc.teamcode.tuning.ConfigStore;
import org.firstinspires.ftc.teamcode.tuning.ConfigValidator;
import org.firstinspires.ftc.teamcode.tuning.Param;
import org.firstinspires.ftc.teamcode.tuning.ParamRegistry;
import org.firstinspires.ftc.teamcode.tuning.RunReport;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * The calibration workflow: load active -> edit a candidate -> run a procedure under active or candidate settings
 * -> compare with the previous run -> save candidate -> explicitly accept or reject. Procedures run with the chosen
 * override set applied on top of the compiled defaults and the previous live values are restored afterwards, so a
 * calibration run can never leave stray settings behind and never writes production configuration by itself.
 */
public final class CalSession {
    private final ConfigStore store;
    private ConfigStore.Config active;
    private final Map<String, Double> candidate = new TreeMap<>();
    private final Map<String, RunReport> baselines = new LinkedHashMap<>();
    private final Map<String, RunReport> lastRun = new LinkedHashMap<>();
    private Map<String, Double> lastSuggestions = new LinkedHashMap<>();

    public CalSession(ConfigStore store) throws IOException {
        this.store = store;
        reloadFromDisk();
    }

    public void reloadFromDisk() throws IOException {
        active = store.loadActive();
        candidate.clear();
        candidate.putAll(store.hasCandidate() ? store.loadCandidate().overrides : active.overrides);
    }

    public ConfigStore store() { return store; }
    public int activeRevision() { return active.revision; }
    public Map<String, Double> candidate() { return new TreeMap<>(candidate); }
    public Map<String, Double> suggestions() { return new LinkedHashMap<>(lastSuggestions); }

    public double candidateValue(String key) { return candidate.containsKey(key) ? candidate.get(key) : ParamRegistry.defaultOf(key); }
    public double activeValue(String key) { return active.overrides.containsKey(key) ? active.overrides.get(key) : ParamRegistry.defaultOf(key); }
    public boolean isExplicit(String key) { return candidate.containsKey(key); }

    /** Manual edit (operator stepping a value). Refuses SAFETY / derived keys and out-of-range values. Returns error or null. */
    public String setManual(String key, double v) {
        Param p = ParamRegistry.get(key);
        if (p == null) return "unknown parameter " + key;
        if (!p.manuallyAdjustable()) return key + " is " + p.category + " and cannot be changed by hand";
        String c = p.check(v);
        if (c != null) return key + ": " + c;
        candidate.put(key, v);
        return null;
    }

    /** Measured value from a procedure (may set SAFETY-category mechanical limits). Range checked; returns error or null. */
    public String setMeasured(String key, double v) {
        Param p = ParamRegistry.get(key);
        if (p == null) return "unknown parameter " + key;
        if (p.category == Param.Category.SAFETY && p.tighten != Param.Tighten.NONE) return key + " is a safety cap, not a measured value";
        if (p.category == Param.Category.FIXED_DERIVED) return key + " is fixed";
        String c = p.check(v);
        if (c != null) return key + ": " + c;
        candidate.put(key, v);
        return null;
    }

    /** Moves the last run's suggestions into the candidate (an explicit operator step). Returns the problems, one per rejected value. */
    public List<String> adoptSuggestions() {
        List<String> problems = new ArrayList<>();
        for (Map.Entry<String, Double> e : lastSuggestions.entrySet()) {
            String err = setMeasured(e.getKey(), e.getValue());
            if (err != null) problems.add(err);
        }
        return problems;
    }

    public List<String> candidateDiff() { return ConfigStore.diff(active.overrides, candidate); }

    /** Revalidates the candidate as a whole (ranges, interlock prerequisites, controller design). */
    public List<String> validateCandidate() { return ConfigValidator.validate(candidate); }

    /**
     * Runs {@code p} under the active settings ({@code useCandidate=false}) or the candidate ({@code true}), saves the
     * report + CSV into the store's runs/ directory and returns the report. The first run of a procedure becomes its
     * baseline; {@link #setBaseline} picks another.
     */
    public RunReport run(java.util.function.Supplier<Procedure> factory, CalContext ctx, boolean useCandidate) throws IOException {
        // The procedure is BUILT inside the applied configuration too, so arguments read from parameters follow the candidate.
        final Procedure probe = factory.get();
        return runInternal(probe, factory, ctx, useCandidate);
    }

    public RunReport run(Procedure p, CalContext ctx, boolean useCandidate) throws IOException {
        return runInternal(p, null, ctx, useCandidate);
    }

    private RunReport runInternal(Procedure p, java.util.function.Supplier<Procedure> factory, CalContext ctx, boolean useCandidate) throws IOException {
        Map<String, Double> overrides = useCandidate ? new TreeMap<>(candidate) : new TreeMap<>(active.overrides);
        List<String> errs = ConfigValidator.validate(overrides);
        RunReport rep = new RunReport(p.name(), ctx.mode());
        rep.baseRevision = active.revision;
        rep.usedCandidate = useCandidate;
        rep.timestamp = System.currentTimeMillis();
        Map<String, Double> effective = ConfigStore.effective(overrides);
        rep.configHash = ConfigStore.hash(effective);
        for (String k : p.paramKeys()) rep.params.put(k, effective.get(k));
        if (!errs.isEmpty()) {
            rep.aborted = true;
            rep.notes.add("REFUSED: configuration invalid");
            rep.notes.addAll(errs);
            lastSuggestions = new LinkedHashMap<>();
            lastRun.put(p.name(), rep);
            return rep;
        }
        if (!baselines.containsKey(p.name())) {
            RunReport earliest = earliestOnDisk(p.name());
            if (earliest != null) baselines.put(p.name(), earliest);
        }
        ProcedureResult res = ParamRegistry.with(overrides, () -> (factory != null ? factory.get() : p).run(ctx));
        rep.aborted = res.aborted || res.refused;
        rep.metrics.putAll(res.metrics);
        rep.notes.addAll(res.notes);
        if (!ctx.mode().equals("HARDWARE")) rep.notes.add("SIMULATION data: does not validate the physical robot");
        if (res.log != null) {
            res.log.meta.put("procedure", p.name());
            res.log.meta.put("mode", ctx.mode());
            res.log.meta.put("baseRevision", Integer.toString(active.revision));
            res.log.meta.put("usedCandidate", Boolean.toString(useCandidate));
            res.log.meta.put("configHash", rep.configHash);
        }
        File f = rep.save(store.runsDir());
        if (res.log != null) res.log.save(new File(store.runsDir(), f.getName().replace(".report.properties", ".csv")));
        lastSuggestions = new LinkedHashMap<>(res.suggestions);
        if (!res.aborted && !res.refused && !baselines.containsKey(p.name())) baselines.put(p.name(), rep);
        lastRun.put(p.name(), rep);
        return rep;
    }

    /** The earliest completed report of this procedure already stored in runs/ (so comparisons survive restarts). */
    private RunReport earliestOnDisk(String procedure) {
        File[] fs = store.runsDir().listFiles((d, n) -> n.startsWith(procedure + "-") && n.endsWith(".report.properties"));
        if (fs == null) return null;
        java.util.Arrays.sort(fs);
        for (File f : fs) {
            try { RunReport r = RunReport.load(f); if (!r.aborted && r.procedure.equals(procedure)) return r; } catch (IOException ignored) { }
        }
        return null;
    }

    public RunReport baseline(String procedure) { return baselines.get(procedure); }
    public RunReport last(String procedure) { return lastRun.get(procedure); }
    public void setBaseline(String procedure, RunReport r) { baselines.put(procedure, r); }

    /** Comparison of the last run against the baseline (empty if there is no earlier run to compare). */
    public List<String> compareToBaseline(String procedure) {
        RunReport b = baselines.get(procedure), c = lastRun.get(procedure);
        if (b == null || c == null || b == c) return new ArrayList<>();
        return RunReport.compare(b, c);
    }

    // ---- explicit configuration actions ----

    public List<String> saveCandidate(String note) throws IOException { return store.saveCandidate(candidate, note); }

    /** EXPLICIT. Saves the candidate (if needed) and makes it the active configuration. */
    public List<String> accept(String note) throws IOException {
        List<String> errs = store.saveCandidate(candidate, note);
        if (!errs.isEmpty()) return errs;
        errs = store.accept(note);
        if (errs.isEmpty()) reloadFromDisk();
        return errs;
    }

    /** EXPLICIT. Discards the candidate and returns to the active settings. */
    public void reject() throws IOException {
        store.reject();
        reloadFromDisk();
    }
}
