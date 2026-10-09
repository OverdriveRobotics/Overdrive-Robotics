package org.firstinspires.ftc.teamcode.tuning;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.TreeMap;

/**
 * Tracks the shooting window from per-loop readiness samples: entries/exits, time inside/outside, readiness delay
 * from the first sample, and which reason blocked shooting how long.
 */
public final class WindowTracker {
    private double t0 = Double.NaN, last = Double.NaN, firstReady = Double.NaN, entered = Double.NaN;
    private boolean ready;
    private String lastReason = "";
    private int entries;
    private double inside, outside, longestWindow;
    private final Map<String, Double> blocked = new LinkedHashMap<>();

    public void sample(double t, boolean isReady, String reason) {
        if (Double.isNaN(t0)) t0 = t;
        if (!Double.isNaN(last)) {
            double dt = t - last;
            if (ready) inside += dt; else { outside += dt; Double d = blocked.get(lastReason); blocked.put(lastReason, (d == null ? 0 : d) + dt); }   // the interval just ended was blocked for the PREVIOUS sample's reason
        }
        if (isReady && !ready) { entries++; entered = t; if (Double.isNaN(firstReady)) firstReady = t - t0; }
        if (!isReady && ready) longestWindow = Math.max(longestWindow, t - entered);
        if (isReady) longestWindow = Math.max(longestWindow, t - entered);
        ready = isReady;
        lastReason = reason;
        last = t;
    }

    public Map<String, Double> metrics() {
        Map<String, Double> m = new TreeMap<>();
        m.put("window_entries", (double) entries);
        m.put("time_in_window_s", inside);
        m.put("time_outside_window_s", outside);
        m.put("readiness_delay_s", firstReady);
        m.put("longest_window_s", longestWindow);
        return m;
    }

    /** Blocking reasons ordered as first seen, with seconds blocked. */
    public Map<String, Double> blockedReasons() { return new LinkedHashMap<>(blocked); }
    public boolean isReady() { return ready; }
}
