package org.firstinspires.ftc.teamcode.tuning;

import java.util.Map;
import java.util.TreeMap;

/** Pure metric calculations over logged signals. All times in seconds. NaN means "not measurable from this data". */
public final class Metrics {
    private Metrics() {}

    /** Metrics whose larger value is better; every other metric is lower-is-better. */
    public static final java.util.Set<String> HIGHER_IS_BETTER = new java.util.HashSet<>(java.util.Arrays.asList(
            "shots_ok", "shot_success_rate", "transfer_success_rate", "time_in_window_s", "aligned_fraction", "window_entries"));

    /**
     * Step-response metrics for output y(t) driven from y0 to {@code target}.
     * rise = 10%->90% of the step; settling = last time outside +-band (measured from t[0]); overshoot in % of the step;
     * steady-state error = mean(y - target) over the final {@code tailFraction}.
     */
    public static Map<String, Double> step(double[] t, double[] y, double target, double band, double tailFraction) {
        Map<String, Double> m = new TreeMap<>();
        int n = t.length;
        if (n < 3) return m;
        double y0 = y[0], size = target - y0, dir = Math.signum(size);
        double r10 = Double.NaN, r90 = Double.NaN, peak = y0, lastOut = Double.NaN;
        boolean everIn = false;
        for (int i = 0; i < n; i++) {
            double frac = (y[i] - y0) / size;
            if (Double.isNaN(r10) && frac >= 0.1) r10 = t[i];
            if (Double.isNaN(r90) && frac >= 0.9) r90 = t[i];
            if (dir * (y[i] - peak) > 0) peak = y[i];
            if (Math.abs(y[i] - target) > band) lastOut = t[i]; else everIn = true;
        }
        double tail0 = t[0] + (t[n - 1] - t[0]) * (1 - tailFraction), sum = 0; int c = 0;
        for (int i = 0; i < n; i++) if (t[i] >= tail0) { sum += y[i] - target; c++; }
        m.put("rise_time_s", Double.isNaN(r10) || Double.isNaN(r90) ? Double.NaN : r90 - r10);
        m.put("settling_time_s", !everIn ? Double.NaN : Double.isNaN(lastOut) ? 0 : lastOut - t[0]);
        m.put("overshoot_pct", size == 0 ? Double.NaN : Math.max(0, dir * (peak - target) / Math.abs(size) * 100));
        m.put("steady_state_error", c == 0 ? Double.NaN : sum / c);
        return m;
    }

    /** RMS and max |ref - y| (tracking error), skipping samples before {@code tSkip}. */
    public static Map<String, Double> tracking(double[] t, double[] ref, double[] y, double tSkip) {
        Map<String, Double> m = new TreeMap<>();
        double ss = 0, mx = 0; int c = 0;
        for (int i = 0; i < t.length; i++) {
            if (t[i] < tSkip || Double.isNaN(ref[i]) || Double.isNaN(y[i])) continue;
            double e = ref[i] - y[i];
            ss += e * e; mx = Math.max(mx, Math.abs(e)); c++;
        }
        m.put("tracking_rms", c == 0 ? Double.NaN : Math.sqrt(ss / c));
        m.put("tracking_max", c == 0 ? Double.NaN : mx);
        return m;
    }

    /** After a disturbance at tDist: size of the dip and time until y is back within +-band of target. */
    public static Map<String, Double> recovery(double[] t, double[] y, double tDist, double target, double band) {
        Map<String, Double> m = new TreeMap<>();
        double minV = Double.POSITIVE_INFINITY, tBack = Double.NaN; boolean dipped = false;
        for (int i = 0; i < t.length; i++) {
            if (t[i] < tDist) continue;
            minV = Math.min(minV, y[i]);
            if (Math.abs(y[i] - target) > band) dipped = true;
            else if (dipped && Double.isNaN(tBack)) tBack = t[i] - tDist;
        }
        m.put("disturbance_drop", Double.isInfinite(minV) ? Double.NaN : target - minV);
        m.put("recovery_time_s", !dipped ? 0 : tBack);
        return m;
    }

    /** Fraction of samples satisfying |v| >= threshold (actuator saturation). */
    public static double fractionAtOrAbove(double[] v, double threshold) {
        if (v.length == 0) return Double.NaN;
        int c = 0;
        for (double x : v) if (Math.abs(x) >= threshold) c++;
        return (double) c / v.length;
    }

    public static int countOutside(double[] v, double lo, double hi) {
        int c = 0;
        for (double x : v) if (x < lo || x > hi) c++;
        return c;
    }

    public static double mean(double[] v) { if (v.length == 0) return Double.NaN; double s = 0; for (double x : v) s += x; return s / v.length; }
    public static double stddev(double[] v) {
        if (v.length < 2) return 0;
        double m = mean(v), s = 0;
        for (double x : v) s += (x - m) * (x - m);
        return Math.sqrt(s / (v.length - 1));
    }
    public static double min(double[] v) { double m = Double.POSITIVE_INFINITY; for (double x : v) m = Math.min(m, x); return m; }
    public static double max(double[] v) { double m = Double.NEGATIVE_INFINITY; for (double x : v) m = Math.max(m, x); return m; }
}
