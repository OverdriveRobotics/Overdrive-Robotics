package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.info;
import static org.firstinspires.ftc.teamcode.T.near;
import static org.firstinspires.ftc.teamcode.T.test;

import java.io.File;
import java.nio.file.Files;
import java.util.List;
import java.util.Map;
import java.util.Random;

/** Metrics, logs, reports, system identification, window tracking and velocity-frame detection on synthetic data. */
public final class MetricsSysIdTests {
    private MetricsSysIdTests() {}

    public static void run() {
        test("metrics: first-order step has rise time 2.197 tau, no overshoot, settles by ~3 tau for a 5% band", () -> {
            double tau = 0.5, target = 1000;
            int n = 400; double[] t = new double[n], y = new double[n];
            for (int i = 0; i < n; i++) { t[i] = i * 0.01; y[i] = target * (1 - Math.exp(-t[i] / tau)); }
            Map<String, Double> m = Metrics.step(t, y, target, 0.05 * target, 0.25);
            near(2.197 * tau, m.get("rise_time_s"), 0.02, "rise");
            near(0, m.get("overshoot_pct"), 1e-9, "overshoot");
            near(-Math.log(0.05) * tau, m.get("settling_time_s"), 0.02, "settling");
            near(0, m.get("steady_state_error"), 0.05 * target, "ss error within the band");
        });

        test("metrics: second-order underdamped step reports the analytic overshoot", () -> {
            double zeta = 0.3, wn = 8, target = 1;
            int n = 1000; double[] t = new double[n], y = new double[n];
            double wd = wn * Math.sqrt(1 - zeta * zeta), phi = Math.acos(zeta);
            for (int i = 0; i < n; i++) {
                t[i] = i * 0.005;
                y[i] = 1 - Math.exp(-zeta * wn * t[i]) / Math.sqrt(1 - zeta * zeta) * Math.sin(wd * t[i] + phi);
            }
            double expected = 100 * Math.exp(-Math.PI * zeta / Math.sqrt(1 - zeta * zeta));
            near(expected, Metrics.step(t, y, target, 0.02, 0.25).get("overshoot_pct"), 0.3, "overshoot %");
        });

        test("metrics: downward steps, tracking error, recovery and saturation", () -> {
            int n = 200; double[] t = new double[n], y = new double[n], ref = new double[n];
            for (int i = 0; i < n; i++) { t[i] = i * 0.01; y[i] = 100 * Math.exp(-t[i] / 0.2); ref[i] = 0; }
            Map<String, Double> m = Metrics.step(t, y, 0, 2, 0.25);
            check(m.get("rise_time_s") > 0.2 && m.get("rise_time_s") < 0.6, "falling step rise: " + m.get("rise_time_s"));
            Map<String, Double> tr = Metrics.tracking(t, ref, y, 1.0);
            check(tr.get("tracking_max") < 1, "late tracking error small");
            // disturbance: dip at t=1 then recover
            double[] tt = new double[300], yy = new double[300];
            for (int i = 0; i < 300; i++) { tt[i] = i * 0.01; yy[i] = 1000; }
            for (int i = 100; i < 140; i++) yy[i] = 1000 - 250 * (1 - (i - 100) / 40.0);
            Map<String, Double> rec = Metrics.recovery(tt, yy, 1.0, 1000, 50);
            near(250, rec.get("disturbance_drop"), 1e-9, "drop");
            near(0.32, rec.get("recovery_time_s"), 0.02, "recovery time");
            near(0.5, Metrics.fractionAtOrAbove(new double[]{1, 1, 0.2, 0.3}, 0.999), 1e-12, "saturation fraction");
            check(Metrics.countOutside(new double[]{-0.1, 0.5, 1.2}, 0, 1) == 2, "constraint violations");
            near(0, Metrics.stddev(new double[]{3, 3, 3}), 1e-12, "encoder consistency");
        });

        test("runlog/report: CSV and report round-trip with metadata; comparison flags better/worse and mode mismatches", () -> {
            File d = Files.createTempDirectory("overdrive-run").toFile();
            RunLog log = new RunLog("t", "y");
            log.meta.put("configHash", "abc"); log.add(0, 1); log.add(1, 2.5);
            File f = new File(d, "x.csv"); log.save(f);
            RunLog back = RunLog.load(f);
            check(back.size() == 2 && back.meta.get("configHash").equals("abc") && back.column("y")[1] == 2.5, "csv round trip");
            RunReport a = new RunReport("flywheel-step", "SIMULATION"); a.configHash = "h1"; a.metrics.put("settling_time_s", 1.0); a.metrics.put("shots_ok", 1.0); a.params.put("flywheel.MODEL_STD", 50.0);
            RunReport b = new RunReport("flywheel-step", "SIMULATION"); b.configHash = "h2"; b.metrics.put("settling_time_s", 0.5); b.metrics.put("shots_ok", 0.0); b.params.put("flywheel.MODEL_STD", 80.0);
            File rf = a.save(d);
            RunReport ra = RunReport.load(rf);
            check(ra.metrics.get("settling_time_s") == 1.0 && ra.params.get("flywheel.MODEL_STD") == 50.0 && ra.configHash.equals("h1"), "report round trip");
            List<String> cmp = RunReport.compare(a, b);
            String all = String.join("\n", cmp);
            check(all.contains("settling_time_s") && all.contains("better"), "lower settling is better:\n" + all);
            check(all.matches("(?s).*shots_ok.*WORSE.*"), "fewer shots is worse");
            check(all.contains("MODEL_STD"), "parameter change listed");
            b.mode = "HARDWARE";
            check(String.join("\n", RunReport.compare(a, b)).contains("different modes"), "mode mismatch warned");
        });

        test("sysid: recovers kS/kV/kA from clean synthetic data (exact within 1%)", () -> {
            double kS = 0.4, kV = 0.0045, kA = 0.002;   // V, V/(tick/s), V/(tick/s^2)
            double a = kV / kA, b = 1 / kA;
            double[][] d = simulate(a, b, kS, 0, 0);
            SysId.Fit f = SysId.fitVelocity(d[0], d[1], d[2]);
            check(f.ok, "fit ok: " + f.problem);
            near(kV, f.kV, kV * 0.01, "kV"); near(kA, f.kA, kA * 0.01, "kA"); near(kS, f.kS, 0.05, "kS");
            check(f.r2 > 0.999, "replay r2 " + f.r2);
        });

        test("sysid: with measurement noise the fit stays close, and bad data is flagged", () -> {
            double kS = 0.3, kV = 0.0042, kA = 0.0025;
            double[][] d = simulate(kV / kA, 1 / kA, kS, 15, 7);
            SysId.Fit f = SysId.fitVelocity(d[0], d[1], d[2]);
            check(f.ok, "fit ok: " + f.problem);
            info(String.format("noisy fit: kV %.5f (true %.5f), kA %.5f (true %.5f), kS %.3f, r2 %.3f", f.kV, kV, f.kA, kA, f.kS, f.r2));
            check(f.r2 > 0.95, "replay r2 stays high despite sample noise: " + f.r2);
            near(kV, f.kV, kV * 0.15, "kV within 15%"); near(kA, f.kA, kA * 0.3, "kA within 30% (differencing noisy velocity biases kA)");
            SysId.Fit bad = SysId.fitVelocity(new double[]{0, 1, 2}, new double[]{1, 1, 1}, new double[]{0, 0, 0});
            check(!bad.ok, "too little data");
            double[] t = new double[100], v = new double[100], w = new double[100];
            for (int i = 0; i < 100; i++) { t[i] = i * 0.02; v[i] = 6; w[i] = 500; }   // velocity constant while volts applied: unidentifiable/non-physical
            check(!SysId.fitVelocity(t, v, w).ok, "non-exciting data refused");
        });

        test("sysid: turret inertia/friction back-computation round-trips through the forward model", () -> {
            double g = 3.5, tau = 2.0, i = 9.0, vn = 12, wf = 30, J = 0.03, bf = 0.02;
            double r = vn / i, kt = tau / i, kv = vn / wf;
            double a = (g * g * kt * kv / r + bf) / J, b = g * kt / (r * J);
            double[] jb = SysId.turretInertiaFriction(a, b, g, tau, i, vn, wf);
            near(J, jb[0], 1e-9, "J"); near(bf, jb[1], 1e-9, "bf");
        });

        test("window tracker: entries, exits, time inside/outside, readiness delay and blocking reasons", () -> {
            WindowTracker w = new WindowTracker();
            for (int i = 0; i <= 100; i++) {
                double t = i * 0.1;
                boolean ready = (t >= 2.0 && t < 4.0) || (t >= 7.0 && t < 8.0);
                w.sample(t, ready, t < 2.0 ? "flywheel not at stable velocity" : "outside shooting zone");
            }
            Map<String, Double> m = w.metrics();
            near(2, m.get("window_entries"), 0, "two windows");
            near(3.0, m.get("time_in_window_s"), 0.11, "inside");
            near(2.0, m.get("readiness_delay_s"), 1e-9, "first ready");
            near(2.0, m.get("longest_window_s"), 0.11, "longest");
            check(w.blockedReasons().get("flywheel not at stable velocity") > 1.8, "reason accounted");
        });

        test("velocity frame detector: separates field-frame from robot-frame velocity (needs heading and motion)", () -> {
            VelocityFrame field = new VelocityFrame(), robot = new VelocityFrame(), still = new VelocityFrame(), headingZero = new VelocityFrame();
            double h = Math.toRadians(90), dt = 0.1;
            for (int i = 0; i < 20; i++) {
                double fx = 20 * Math.cos(i * 0.3), fy = 20 * Math.sin(i * 0.3);   // true field velocity
                double dx = fx * dt, dy = fy * dt;
                field.add(h, fx, fy, dx, dy, dt);                                  // reports the field velocity
                double c = Math.cos(h), s = Math.sin(h);
                robot.add(h, fx * c + fy * s, -fx * s + fy * c, dx, dy, dt);       // reports robot-frame velocity
                still.add(h, 0.1, 0.1, 0.01, 0.01, dt);
                headingZero.add(0, fx, fy, dx, dy, dt);
            }
            check(field.verdict() == VelocityFrame.Verdict.FIELD, "field");
            check(robot.verdict() == VelocityFrame.Verdict.ROBOT, "robot");
            check(still.verdict() == VelocityFrame.Verdict.UNDETERMINED, "no motion -> undetermined");
            check(headingZero.verdict() == VelocityFrame.Verdict.UNDETERMINED, "heading 0 -> frames coincide -> undetermined");
        });
    }

    /** Positive-direction voltage steps + coast, using the exact model; optional gaussian noise on omega. */
    static double[][] simulate(double a, double b, double kS, double noise, long seed) {
        Random rng = new Random(seed);
        double dt = 0.02, w = 0;
        int n = 0; double[] t = new double[2000], v = new double[2000], om = new double[2000];
        double[] volts = {4, 7, 10};
        double time = 0;
        for (double vs : volts) for (int i = 0; i < 150; i++) { w = step(w, vs, a, b, kS, dt); t[n] = time += dt; v[n] = vs; om[n] = w + rng.nextGaussian() * noise; n++; }
        for (int i = 0; i < 200; i++) { w = step(w, 0, a, b, kS, dt); t[n] = time += dt; v[n] = 0; om[n] = w + rng.nextGaussian() * noise; n++; }
        return new double[][]{java.util.Arrays.copyOf(t, n), java.util.Arrays.copyOf(v, n), java.util.Arrays.copyOf(om, n)};
    }

    private static double step(double w, double volts, double a, double b, double kS, double dt) {
        double eff = volts > 0 ? Math.max(0, volts - kS) : 0;
        double e = Math.exp(-a * dt);
        return e * w + b * (1 - e) / a * eff;
    }
}
