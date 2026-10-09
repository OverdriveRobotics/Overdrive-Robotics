package org.firstinspires.ftc.teamcode.tuning;

import org.firstinspires.ftc.teamcode.control.AngleUtil;
import org.firstinspires.ftc.teamcode.control.FlywheelConfig;
import org.firstinspires.ftc.teamcode.control.FlywheelLqrController;
import org.firstinspires.ftc.teamcode.control.TurretStateSpaceController;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.TreeMap;

/**
 * Offline grid search of controller weights on a SIMULATED plant. It ranks candidates under an assumed model with
 * +-15% parameter mismatch, measurement noise and a disturbance. It does NOT establish optimal physical parameters:
 * the plant is whatever the current configuration says (identified values if you have them, placeholders if not).
 * Output is a candidate to be reviewed, run on the robot and explicitly accepted - never applied automatically.
 */
public final class OfflineTuner {
    private OfflineTuner() {}

    public static final String DISCLAIMER = "SIMULATION: ranks candidates on the configured model (+-15% mismatch, noise, one disturbance). "
            + "Not a physical optimum; verify on the robot before accepting.";

    public static final class Result {
        public final Map<String, Double> best = new TreeMap<>();
        public double bestScore, defaultScore;
        public final List<String> table = new ArrayList<>();
        public String disclaimer = DISCLAIMER;
    }

    private static final double[] MISMATCH = {1.0, 0.85, 1.15};

    // ------------------------------------------------------------------ flywheel

    /** Score (lower is better) of one weight pair over all mismatch scenarios. */
    public static double scoreFlywheel(FlywheelLqrController.Params nominal, double velErr, double modelStd, double target, long seed) {
        double total = 0;
        for (double mm : MISMATCH) {
            FlywheelLqrController.Params p = copy(nominal);
            p.maxVelocityError = velErr; p.modelStd = modelStd;
            FlywheelLqrController ctl = new FlywheelLqrController(p);
            Random rng = new Random(seed);
            double a = nominal.a * mm, b = nominal.b / mm, w = 0, dt = 0.02, vbat = 12;
            int n = 600; double[] t = new double[n], y = new double[n], u = new double[n];
            double tDist = 8.0;
            for (int i = 0; i < n; i++) {
                if (i == (int) (tDist / dt)) w -= 250;
                double meas = w + rng.nextGaussian() * 20;
                double power = ctl.update(target, meas, vbat, dt);
                double e = Math.exp(-a * dt);
                w = e * w + b * (1 - e) / a * (power * vbat);
                t[i] = i * dt; y[i] = w; u[i] = power;
            }
            double band = 50;
            Map<String, Double> st = Metrics.step(t, y, target, band, 0.2);
            Map<String, Double> rc = Metrics.recovery(t, y, tDist, target, band);
            double settle = nz(st.get("settling_time_s"), 12), os = nz(st.get("overshoot_pct"), 50);
            double rec = nz(rc.get("recovery_time_s"), 4), ss = Math.abs(nz(st.get("steady_state_error"), 500));
            total += settle + os / 20.0 + 2.0 * rec + ss / band + 5.0 * Metrics.fractionAtOrAbove(u, 0.999);
        }
        return total / MISMATCH.length;
    }

    public static Result tuneFlywheel(FlywheelLqrController.Params nominal, double target, long seed) {
        double[] velErr = {25, 50, 100, 200, 400, 800};
        double[] modelStd = {10, 25, 50, 100, 200, 400};
        Result r = new Result();
        r.defaultScore = scoreFlywheel(nominal, nominal.maxVelocityError, nominal.modelStd, target, seed);
        r.bestScore = Double.POSITIVE_INFINITY;
        r.table.add(String.format("%-14s %-12s %s", "MAX_VEL_ERR", "MODEL_STD", "score (lower is better)"));
        for (double ve : velErr) for (double ms : modelStd) {
            double s = scoreFlywheel(nominal, ve, ms, target, seed);
            r.table.add(String.format("%-14.0f %-12.0f %.3f", ve, ms, s));
            if (s < r.bestScore) {
                r.bestScore = s; r.best.clear();
                r.best.put("flywheel.MAX_VELOCITY_ERROR", ve); r.best.put("flywheel.MODEL_STD", ms);
            }
        }
        r.table.add(String.format("default (%.0f, %.0f): %.3f   best: %.3f", nominal.maxVelocityError, nominal.modelStd, r.defaultScore, r.bestScore));
        return r;
    }

    // ------------------------------------------------------------------ turret

    public static double scoreTurret(TurretStateSpaceController.Params nominal, double angErr, double velErr, double refVel, long seed) {
        double total = 0;
        for (double mm : MISMATCH) {
            TurretStateSpaceController.Params p = copy(nominal);
            p.maxAngleError = angErr; p.maxVelocityError = velErr; p.maxRefVelocity = refVel;
            TurretStateSpaceController ctl = new TurretStateSpaceController(p);
            Random rng = new Random(seed);
            double a = nominal.a * mm, b = nominal.b / mm, th = 0, om = 0, dt = nominal.samplePeriod, vbat = 12;
            int n = (int) (8.0 / dt);
            double[] t = new double[n], y = new double[n], ref = new double[n], u = new double[n];
            for (int i = 0; i < n; i++) {
                double time = i * dt;
                double goal = time < 3 ? Math.toRadians(45) : Math.toRadians(-30) + Math.toRadians(20) * (time - 3) * 0.2;   // step, then a slow ramp
                double goalVel = time < 3 ? 0 : Math.toRadians(20) * 0.2;
                double power = ctl.update(goal, goalVel, th + rng.nextGaussian() * 0.0005, om + rng.nextGaussian() * 0.02, vbat, dt);
                double v = Math.max(-1, Math.min(1, power)) * vbat, e = Math.exp(-a * dt);
                double nom = e * om + b * (1 - e) / a * v;
                th += om * (1 - e) / a + b * v * (dt - (1 - e) / a) / a;
                om = nom;
                t[i] = time; y[i] = th; ref[i] = goal; u[i] = power;
            }
            Map<String, Double> st = Metrics.step(java.util.Arrays.copyOfRange(t, 0, (int) (3 / dt)), java.util.Arrays.copyOfRange(y, 0, (int) (3 / dt)),
                    Math.toRadians(45), Math.toRadians(1.5), 0.2);
            Map<String, Double> tr = Metrics.tracking(t, ref, y, 4.0);
            total += nz(st.get("settling_time_s"), 3) + nz(st.get("overshoot_pct"), 30) / 20.0
                    + Math.toDegrees(nz(tr.get("tracking_rms"), 1)) + 5.0 * Metrics.fractionAtOrAbove(u, 0.999);
        }
        return total / MISMATCH.length;
    }

    public static Result tuneTurret(TurretStateSpaceController.Params nominal, long seed) {
        double[] ang = {Math.toRadians(2), Math.toRadians(5), Math.toRadians(10), Math.toRadians(20)};
        double[] vel = {0.25, 0.5, 1, 2};
        double[] ref = {3, 6, 10};
        Result r = new Result();
        r.defaultScore = scoreTurret(nominal, nominal.maxAngleError, nominal.maxVelocityError, nominal.maxRefVelocity, seed);
        r.bestScore = Double.POSITIVE_INFINITY;
        r.table.add(String.format("%-16s %-16s %-12s %s", "MAX_ANGLE_ERR", "MAX_VEL_ERR", "MAX_REF_VEL", "score (lower is better)"));
        for (double a : ang) for (double v : vel) for (double rv : ref) {
            double s = scoreTurret(nominal, a, v, rv, seed);
            r.table.add(String.format("%-16.4f %-16.2f %-12.1f %.3f", a, v, rv, s));
            if (s < r.bestScore) {
                r.bestScore = s; r.best.clear();
                r.best.put("turret.MAX_ANGLE_ERROR_RAD", a); r.best.put("turret.MAX_VELOCITY_ERROR_RAD_S", v);
                r.best.put("turret.MAX_REFERENCE_VELOCITY_RAD_S", rv);
            }
        }
        r.table.add(String.format("default: %.3f   best: %.3f", r.defaultScore, r.bestScore));
        return r;
    }

    private static double nz(Double v, double fallback) { return v == null || Double.isNaN(v) ? fallback : v; }

    private static FlywheelLqrController.Params copy(FlywheelLqrController.Params s) {
        FlywheelLqrController.Params p = new FlywheelLqrController.Params();
        p.a = s.a; p.b = s.b; p.kS = s.kS; p.samplePeriod = s.samplePeriod; p.maxVelocityError = s.maxVelocityError;
        p.maxVolts = s.maxVolts; p.modelStd = s.modelStd; p.measurementStd = s.measurementStd;
        return p;
    }

    private static TurretStateSpaceController.Params copy(TurretStateSpaceController.Params s) {
        TurretStateSpaceController.Params p = new TurretStateSpaceController.Params();
        p.a = s.a; p.b = s.b; p.samplePeriod = s.samplePeriod; p.maxAngleError = s.maxAngleError;
        p.maxVelocityError = s.maxVelocityError; p.maxVolts = s.maxVolts; p.maxRefVelocity = s.maxRefVelocity;
        p.velocityAlpha = s.velocityAlpha; p.staticFrictionVolts = s.staticFrictionVolts; p.integralGain = s.integralGain;
        p.integralLimitVolts = s.integralLimitVolts;
        return p;
    }

    @SuppressWarnings("unused") private static void unused(LinkedHashMap<String, String> m) { AngleUtil.wrap(0); FlywheelConfig.class.getName(); }
}
