package org.firstinspires.ftc.teamcode.tuning;

import org.firstinspires.ftc.teamcode.control.Matrices;

/**
 * Least-squares system identification from logged data (no hidden constants: everything comes from the log).
 * Model of a velocity-controlled mechanism: w' = -a*w + b*(V - kS) while driven (V &gt; 0),  w' = -a*w when V = 0.
 * LOG CONVENTION: row k holds (t[k], volts[k], w[k]) where volts[k] is the voltage applied over the interval that
 * ENDS at t[k] (command, loop, then read), so w[k]->w[k+1] is driven by volts[k+1].
 * Exact zero-order-hold discretisation: w[k+1] = alpha*w[k] + beta*V + gamma (V &gt; 0), alpha = e^(-a dt),
 * beta = b(1-alpha)/a, gamma = -beta*kS, fitted by ordinary least squares; the sample period is the mean of the log.
 * Noise on w[k] (a regressor) attenuates alpha toward 0 (errors-in-variables), so noisy data biases a upward:
 * {@code r2}/{@code rmse} are computed by REPLAYING the fitted model over the recorded voltages (whole trajectory), which is
 * the validation the WPILib tutorial recommends; still confirm with the step-response procedure before relying on it.
 */
public final class SysId {
    private SysId() {}

    public static final class Fit {
        public double a, b, kS, kV, kA, rmse, r2;
        public int samples;
        public boolean ok;
        /** False when only one drive voltage was used: kS and the gain are then not separable and kS is reported as 0. */
        public boolean kSIdentified = true;
        public String problem;
    }

    public static Fit fitVelocity(double[] t, double[] volts, double[] w) {
        Fit f = new Fit();
        int n = t.length;
        if (volts.length != n || w.length != n || n < 20) { f.problem = "need at least 20 samples"; return f; }
        // mean sample period over usable steps (the discrete model assumes a constant period)
        double dtSum = 0; int dtN = 0;
        for (int k = 0; k + 1 < n; k++) {
            double dt = t[k + 1] - t[k];
            if (dt > 1e-4 && dt <= 0.25 && !Double.isNaN(w[k]) && !Double.isNaN(w[k + 1])) { dtSum += dt; dtN++; }
        }
        if (dtN < 20) { f.problem = "too few usable samples"; return f; }
        double dtBar = dtSum / dtN;
        double vMin = Double.POSITIVE_INFINITY, vMax = 0;
        for (int k = 1; k < n; k++) if (volts[k] > 1e-6) { vMin = Math.min(vMin, volts[k]); vMax = Math.max(vMax, volts[k]); }
        boolean useGamma = vMax > 0 && (vMax - vMin) >= 0.15 * vMax;
        f.kSIdentified = useGamma;
        double[][] ata = new double[3][3]; double[] atb = new double[3];
        boolean driven = false, coast = false;
        for (int k = 0; k + 1 < n; k++) {
            double dt = t[k + 1] - t[k];
            if (!(dt > 1e-4) || dt > 0.25 || Double.isNaN(w[k]) || Double.isNaN(w[k + 1])) continue;
            boolean on = volts[k + 1] > 1e-6;
            double[] x = {w[k], on ? volts[k + 1] : 0, (on && useGamma) ? 1 : 0};   // w[k+1] = alpha*w[k] + beta*V + gamma, gamma = -beta*kS
            driven |= on; coast |= !on;
            for (int i = 0; i < 3; i++) { for (int j = 0; j < 3; j++) ata[i][j] += x[i] * x[j]; atb[i] += x[i] * w[k + 1]; }
        }
        if (!driven) { f.problem = "no driven samples (V > 0)"; return f; }
        if (!coast || !useGamma) ata[2][2] += 1e-9;
        double[] th = new double[3];
        try {
            double[][] inv = Matrices.inverse(ata);
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) th[i] += inv[i][j] * atb[j];
        } catch (ArithmeticException e) { f.problem = "data not exciting enough (singular regression)"; return f; }
        double alpha = th[0], beta = th[1];
        if (!useGamma) th[2] = 0;
        if (!(alpha > 0 && alpha < 1) || !(beta > 0)) { f.problem = "non-physical fit (alpha=" + alpha + ", beta=" + beta + "): check units/logging/excitation"; return f; }
        f.a = -Math.log(alpha) / dtBar;
        f.b = beta * f.a / (1 - alpha);
        f.kS = Math.max(0, -th[2] / beta);
        f.kV = f.a / f.b; f.kA = 1 / f.b;
        // Goodness of fit = REPLAY: simulate the fitted model forward from the first sample using only the recorded voltages
        // and compare the whole velocity trajectory (robust to sample noise, unlike one-step increments).
        double ss = 0, sTot = 0, mean = 0; int m = 0;
        for (int k = 0; k < n; k++) if (!Double.isNaN(w[k])) { mean += w[k]; m++; }
        mean /= m;
        double sim = w[0];
        for (int k = 1; k < n; k++) {
            if (Double.isNaN(w[k])) continue;
            boolean on = volts[k] > 1e-6;
            sim = alpha * sim + (on ? beta * volts[k] + th[2] : 0);
            ss += (w[k] - sim) * (w[k] - sim);
            sTot += (w[k] - mean) * (w[k] - mean);
        }
        f.rmse = Math.sqrt(ss / m);
        f.r2 = sTot > 0 ? 1 - ss / sTot : Double.NaN;
        f.samples = m;
        f.ok = true;
        return f;
    }

    /**
     * Turns a fitted turret model (a, b) back into the inertia and viscous friction the config stores, given the
     * (explicit!) motor constants: J = G*Kt/(R*b),  bf = a*J - G^2*Kt*Kv/R.  Returns {J, bf} or null if non-physical.
     */
    public static double[] turretInertiaFriction(double a, double b, double g, double stallTorque, double stallCurrent,
                                                 double nominalVolts, double freeSpeed) {
        double r = nominalVolts / stallCurrent, kt = stallTorque / stallCurrent, kv = nominalVolts / freeSpeed;
        double j = g * kt / (r * b);
        double bf = a * j - g * g * kt * kv / r;
        if (!(j > 0)) return null;
        return new double[]{j, Math.max(0, bf)};
    }
}
