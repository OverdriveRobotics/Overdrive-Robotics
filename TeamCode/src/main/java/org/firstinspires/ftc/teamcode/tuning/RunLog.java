package org.firstinspires.ftc.teamcode.tuning;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Timestamped numeric log with a fixed set of columns, saved as CSV with '# key=value' metadata lines. */
public final class RunLog {
    public final String[] columns;
    public final Map<String, String> meta = new LinkedHashMap<>();
    private final List<double[]> rows = new ArrayList<>();

    public RunLog(String... columns) { this.columns = columns; }

    public void add(double... values) {
        if (values.length != columns.length) throw new IllegalArgumentException("expected " + columns.length + " values");
        rows.add(values.clone());
    }

    public int size() { return rows.size(); }

    public int col(String name) {
        for (int i = 0; i < columns.length; i++) if (columns[i].equals(name)) return i;
        throw new IllegalArgumentException("no column " + name);
    }

    public double[] column(String name) {
        int c = col(name);
        double[] a = new double[rows.size()];
        for (int i = 0; i < a.length; i++) a[i] = rows.get(i)[c];
        return a;
    }

    /** Sub-log of rows whose first column (time) lies in [t0, t1]. */
    public RunLog window(double t0, double t1) {
        RunLog r = new RunLog(columns);
        r.meta.putAll(meta);
        for (double[] row : rows) if (row[0] >= t0 && row[0] <= t1) r.rows.add(row);
        return r;
    }

    public void save(File f) throws IOException {
        f.getParentFile().mkdirs();
        StringBuilder sb = new StringBuilder();
        for (Map.Entry<String, String> e : meta.entrySet()) sb.append("# ").append(e.getKey()).append('=').append(e.getValue()).append('\n');
        sb.append(String.join(",", columns)).append('\n');
        for (double[] r : rows) {
            for (int i = 0; i < r.length; i++) { if (i > 0) sb.append(','); sb.append(r[i]); }
            sb.append('\n');
        }
        Files.write(f.toPath(), sb.toString().getBytes(StandardCharsets.UTF_8));
    }

    public static RunLog load(File f) throws IOException {
        Map<String, String> meta = new LinkedHashMap<>();
        RunLog log = null;
        try (BufferedReader r = Files.newBufferedReader(f.toPath(), StandardCharsets.UTF_8)) {
            String line;
            while ((line = r.readLine()) != null) {
                if (line.startsWith("# ")) { int i = line.indexOf('='); if (i > 0) meta.put(line.substring(2, i), line.substring(i + 1)); continue; }
                if (line.trim().isEmpty()) continue;
                if (log == null) { log = new RunLog(line.split(",")); log.meta.putAll(meta); continue; }
                String[] p = line.split(",");
                double[] v = new double[p.length];
                for (int i = 0; i < p.length; i++) v[i] = Double.parseDouble(p[i]);
                log.add(v);
            }
        }
        if (log == null) throw new IOException(f.getName() + ": empty log");
        return log;
    }
}
