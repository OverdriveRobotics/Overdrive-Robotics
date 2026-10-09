package org.firstinspires.ftc.teamcode.tuning;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.security.MessageDigest;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.TreeMap;

/**
 * File-backed configuration with an explicit workflow: active -> candidate -> accept | reject, with history and
 * restore. Nothing here runs implicitly: ordinary tests and calibration runs only ever write candidates and run
 * reports. Layout under {@code dir}:
 * <pre>
 *   active.properties        the accepted overrides (loaded by RobotHardware.init)
 *   candidate.properties     proposal waiting for accept/reject
 *   history/rev-0003.properties   every accepted revision (never overwritten)
 *   known-good.txt           revisions an operator explicitly marked as verified
 *   accepted.log, rejected/  audit trail
 *   runs/                    run reports + CSV logs
 * </pre>
 * Files are sparse: only parameters that differ from the compiled defaults are stored, so "explicitly calibrated"
 * is exactly "present in the file".
 */
public final class ConfigStore {
    public static final int SCHEMA = 1;

    /** A parsed configuration file. */
    public static final class Config {
        public final int revision, parent;
        public final String note, savedAt;
        public final Map<String, Double> overrides;
        Config(int revision, int parent, String note, String savedAt, Map<String, Double> o) {
            this.revision = revision; this.parent = parent; this.note = note; this.savedAt = savedAt; this.overrides = o;
        }
        public String hash() { return ConfigStore.hash(effective(overrides)); }
    }

    private final File dir;
    private final java.util.function.LongSupplier clock;

    public ConfigStore(File dir) { this(dir, System::currentTimeMillis); }
    public ConfigStore(File dir, java.util.function.LongSupplier clockMs) { this.dir = dir; this.clock = clockMs; }

    public File dir() { return dir; }
    public File runsDir() { return new File(dir, "runs"); }
    private File active() { return new File(dir, "active.properties"); }
    private File candidate() { return new File(dir, "candidate.properties"); }
    private File history(int rev) { return new File(new File(dir, "history"), String.format("rev-%04d.properties", rev)); }

    // ------------------------------------------------------------------ read

    public Config loadActive() throws IOException { return active().exists() ? read(active()) : new Config(0, 0, "defaults", "", new TreeMap<String, Double>()); }
    public boolean hasCandidate() { return candidate().exists(); }
    public Config loadCandidate() throws IOException { return read(candidate()); }
    public Config loadRevision(int rev) throws IOException { return read(history(rev)); }

    public List<Integer> revisions() {
        List<Integer> l = new ArrayList<>();
        File[] fs = new File(dir, "history").listFiles();
        if (fs != null) for (File f : fs) {
            String n = f.getName();
            if (n.startsWith("rev-") && n.endsWith(".properties")) l.add(Integer.parseInt(n.substring(4, n.length() - 11)));
        }
        java.util.Collections.sort(l);
        return l;
    }

    public List<Integer> knownGood() throws IOException {
        List<Integer> l = new ArrayList<>();
        File f = new File(dir, "known-good.txt");
        if (f.exists()) for (String s : Files.readAllLines(f.toPath(), StandardCharsets.UTF_8)) if (!s.trim().isEmpty()) l.add(Integer.parseInt(s.trim()));
        return l;
    }

    // ------------------------------------------------------------------ write

    /** Writes a candidate (the FULL desired override set). Refuses invalid values: returns the errors, writes nothing. */
    public List<String> saveCandidate(Map<String, Double> overrides, String note) throws IOException {
        List<String> errs = ConfigValidator.validate(overrides);
        if (!errs.isEmpty()) return errs;
        Config a = loadActive();
        write(candidate(), a.revision + 1, a.revision, note, overrides);
        return errs;
    }

    /** Starts a candidate as a copy of the active set (no-op if one exists). */
    public Map<String, Double> candidateOrActive() throws IOException {
        return new TreeMap<>(hasCandidate() ? loadCandidate().overrides : loadActive().overrides);
    }

    /**
     * EXPLICIT acceptance: validates the candidate again, archives it as the next revision and makes it active.
     * Returns validation errors (nothing changes) or an empty list on success.
     */
    public List<String> accept(String note) throws IOException {
        if (!hasCandidate()) return java.util.Collections.singletonList("no candidate to accept");
        Config c = loadCandidate();
        List<String> errs = ConfigValidator.validate(c.overrides);
        if (!errs.isEmpty()) return errs;
        Config a = loadActive();
        if (a.revision != c.parent) return java.util.Collections.singletonList(
                "candidate was based on revision " + c.parent + " but active is revision " + a.revision + " - recreate the candidate");
        int rev = a.revision + 1;
        write(history(rev), rev, a.revision, note != null ? note : c.note, c.overrides);
        write(active(), rev, a.revision, note != null ? note : c.note, c.overrides);
        appendLog("accepted rev " + rev + " (parent " + a.revision + "): " + (note != null ? note : c.note) + "\n    "
                + String.join("\n    ", diff(a.overrides, c.overrides)));
        Files.delete(candidate().toPath());
        return errs;
    }

    /** Discards the candidate, keeping a copy under rejected/ for the record. */
    public boolean reject() throws IOException {
        if (!hasCandidate()) return false;
        File rd = new File(dir, "rejected");
        rd.mkdirs();
        Files.move(candidate().toPath(), new File(rd, "candidate-" + clock.getAsLong() + ".properties").toPath(), StandardCopyOption.REPLACE_EXISTING);
        appendLog("rejected candidate");
        return true;
    }

    /** Marks an archived revision as verified on the robot. Explicit operator action. */
    public void markKnownGood(int rev) throws IOException {
        if (!history(rev).exists()) throw new IOException("no such revision " + rev);
        if (knownGood().contains(rev)) return;
        Files.write(new File(dir, "known-good.txt").toPath(), (rev + "\n").getBytes(StandardCharsets.UTF_8),
                java.nio.file.StandardOpenOption.CREATE, java.nio.file.StandardOpenOption.APPEND);
        appendLog("marked rev " + rev + " known-good");
    }

    /** Restores an archived revision as a NEW active revision (history is never rewritten). */
    public List<String> restore(int rev) throws IOException {
        if (!history(rev).exists()) return java.util.Collections.singletonList("no such revision " + rev);
        Config old = loadRevision(rev), a = loadActive();
        List<String> errs = ConfigValidator.validate(old.overrides);
        if (!errs.isEmpty()) return errs;
        int n = a.revision + 1;
        String note = "restore of rev " + rev;
        write(history(n), n, a.revision, note, old.overrides);
        write(active(), n, a.revision, note, old.overrides);
        appendLog("restored rev " + rev + " as rev " + n);
        return errs;
    }

    public List<String> restoreLatestKnownGood() throws IOException {
        List<Integer> kg = knownGood();
        if (kg.isEmpty()) return java.util.Collections.singletonList("no known-good revision recorded");
        return restore(kg.get(kg.size() - 1));
    }

    // ------------------------------------------------------------------ helpers

    /** Effective values = compiled defaults overlaid with the overrides. */
    public static Map<String, Double> effective(Map<String, Double> overrides) {
        Map<String, Double> m = new TreeMap<>();
        for (String k : ParamRegistry.all().keySet()) m.put(k, ParamRegistry.defaultOf(k));
        m.putAll(overrides);
        return m;
    }

    public static String hash(Map<String, Double> effective) {
        try {
            MessageDigest md = MessageDigest.getInstance("SHA-256");
            for (Map.Entry<String, Double> e : new TreeMap<>(effective).entrySet())
                md.update((e.getKey() + "=" + e.getValue() + "\n").getBytes(StandardCharsets.UTF_8));
            StringBuilder sb = new StringBuilder();
            byte[] d = md.digest();
            for (int i = 0; i < 6; i++) sb.append(String.format("%02x", d[i]));
            return sb.toString();
        } catch (java.security.NoSuchAlgorithmException e) { throw new IllegalStateException(e); }
    }

    /** Human-readable differences between two override sets (against the compiled default where absent). */
    public static List<String> diff(Map<String, Double> a, Map<String, Double> b) {
        List<String> out = new ArrayList<>();
        Map<String, Double> ea = effective(a), eb = effective(b);
        for (String k : ea.keySet()) {
            double x = ea.get(k), y = eb.get(k);
            if (!(Double.isNaN(x) && Double.isNaN(y)) && x != y) out.add(k + ": " + x + " -> " + y);
        }
        return out;
    }

    private void appendLog(String line) throws IOException {
        dir.mkdirs();
        Files.write(new File(dir, "accepted.log").toPath(), (clock.getAsLong() + " " + line + "\n").getBytes(StandardCharsets.UTF_8),
                java.nio.file.StandardOpenOption.CREATE, java.nio.file.StandardOpenOption.APPEND);
    }

    private void write(File f, int rev, int parent, String note, Map<String, Double> o) throws IOException {
        f.getParentFile().mkdirs();
        StringBuilder sb = new StringBuilder();
        sb.append("# overdrive robot configuration (sparse overrides on compiled defaults)\n");
        sb.append("schema=").append(SCHEMA).append('\n');
        sb.append("revision=").append(rev).append('\n');
        sb.append("parent=").append(parent).append('\n');
        sb.append("savedAt=").append(clock.getAsLong()).append('\n');
        sb.append("note=").append(note == null ? "" : note.replace('\n', ' ')).append('\n');
        for (Map.Entry<String, Double> e : new TreeMap<>(o).entrySet())
            sb.append("p.").append(e.getKey()).append('=').append(e.getValue()).append('\n');
        File tmp = new File(f.getParentFile(), f.getName() + ".tmp");
        Files.write(tmp.toPath(), sb.toString().getBytes(StandardCharsets.UTF_8));
        Files.move(tmp.toPath(), f.toPath(), StandardCopyOption.REPLACE_EXISTING);
    }

    private Config read(File f) throws IOException {
        Properties p = new Properties();
        try (java.io.Reader r = new java.io.InputStreamReader(new java.io.FileInputStream(f), StandardCharsets.UTF_8)) { p.load(r); }
        String schema = p.getProperty("schema");
        if (!String.valueOf(SCHEMA).equals(schema)) throw new IOException(f.getName() + ": unsupported schema " + schema);
        Map<String, Double> o = new TreeMap<>();
        for (String name : p.stringPropertyNames()) {
            if (!name.startsWith("p.")) continue;
            try { o.put(name.substring(2), Double.parseDouble(p.getProperty(name).trim())); }
            catch (NumberFormatException e) { throw new IOException(f.getName() + ": bad number for " + name); }
        }
        return new Config(Integer.parseInt(p.getProperty("revision", "0")), Integer.parseInt(p.getProperty("parent", "0")),
                p.getProperty("note", ""), p.getProperty("savedAt", ""), o);
    }
}
