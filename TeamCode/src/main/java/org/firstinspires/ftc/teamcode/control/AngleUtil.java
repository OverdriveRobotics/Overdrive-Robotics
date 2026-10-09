package org.firstinspires.ftc.teamcode.control;

/** Angle helpers (radians). Pure math: no FTC dependencies. */
public final class AngleUtil {
    private AngleUtil() {}

    private static final double TWO_PI = 2 * Math.PI;

    /** Wraps to (-PI, PI]. */
    public static double wrap(double a) {
        if (!Double.isFinite(a)) return Double.NaN;
        double r = a - TWO_PI * Math.floor((a + Math.PI) / TWO_PI);   // [-PI, PI)
        return r == -Math.PI ? Math.PI : r;
    }

    /** Signed shortest rotation from {@code current} to {@code target}, in (-PI, PI]. */
    public static double shortestError(double target, double current) {
        return wrap(target - current);
    }

    /**
     * Among the angles {@code desired + 2*PI*k} that lie inside [min, max], returns the one closest to
     * {@code reference} (the measured turret angle), or NaN if none is reachable. For a continuous turret pass
     * infinite limits: the result is then the shortest path from {@code reference}.
     */
    public static double equivalentInRange(double desired, double reference, double min, double max) {
        if (!Double.isFinite(desired) || !Double.isFinite(reference) || Double.isNaN(min) || Double.isNaN(max)
                || min > max) return Double.NaN;
        double best = Double.NaN;
        // Candidate nearest to the reference first, then walk outward to find reachable ones.
        double k0 = Math.round((reference - desired) / TWO_PI);
        for (int dk = -2; dk <= 2; dk++) {
            double c = desired + TWO_PI * (k0 + dk);
            if (c < min - 1e-12 || c > max + 1e-12) continue;
            if (Double.isNaN(best) || Math.abs(c - reference) < Math.abs(best - reference)) best = c;
        }
        return best;
    }

    public static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
