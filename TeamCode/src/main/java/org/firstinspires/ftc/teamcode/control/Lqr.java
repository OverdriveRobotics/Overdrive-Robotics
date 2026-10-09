package org.firstinspires.ftc.teamcode.control;

/**
 * Discrete-time LQR / steady-state Kalman design by iterating the discrete algebraic Riccati equation
 * (DARE) to convergence. Intended to run ONCE at construction; gains are then constants.
 */
public final class Lqr {
    private Lqr() {}

    /**
     * Minimises sum(x'Qx + u'Ru) for x[k+1] = Ad x[k] + Bd u[k]. Returns K (m x n) for u = -K x.
     * Throws if (Ad, Bd) is not controllable or the iteration does not converge.
     */
    public static double[][] dlqr(double[][] ad, double[][] bd, double[][] q, double[][] r) {
        if (!Matrices.isControllable(ad, bd)) throw new IllegalArgumentException("(A,B) is not controllable");
        double[][] at = Matrices.transpose(ad), bt = Matrices.transpose(bd);
        double[][] p = Matrices.copy(q);
        for (int it = 0; it < 200000; it++) {
            double[][] btp = Matrices.mul(bt, p);
            double[][] s = Matrices.add(r, Matrices.mul(btp, bd));
            double[][] k = Matrices.mul(Matrices.inverse(s), Matrices.mul(btp, ad));
            double[][] next = Matrices.add(q, Matrices.sub(Matrices.mul(Matrices.mul(at, p), ad),
                    Matrices.mul(Matrices.mul(Matrices.mul(at, p), bd), k)));
            double change = Matrices.maxAbs(Matrices.sub(next, p));
            p = next;
            if (change < 1e-12 * Math.max(1.0, Matrices.maxAbs(p))) {
                btp = Matrices.mul(bt, p);
                s = Matrices.add(r, Matrices.mul(btp, bd));
                return Matrices.mul(Matrices.inverse(s), Matrices.mul(btp, ad));
            }
        }
        throw new ArithmeticException("DARE did not converge");
    }

    /**
     * Steady-state Kalman gain L (n x p) for x[k+1] = A x + w (cov Qw), y = C x + v (cov Rv), found by iterating
     * the filter-form Riccati recursion. Correct step: xhat = xpred + L (y - C xpred).
     */
    public static double[][] steadyStateKalman(double[][] a, double[][] c, double[][] qw, double[][] rv) {
        double[][] at = Matrices.transpose(a), ct = Matrices.transpose(c);
        int n = a.length;
        double[][] p = Matrices.identity(n);
        for (int it = 0; it < 200000; it++) {
            double[][] pPred = Matrices.add(Matrices.mul(Matrices.mul(a, p), at), qw);
            double[][] s = Matrices.add(Matrices.mul(Matrices.mul(c, pPred), ct), rv);
            double[][] l = Matrices.mul(Matrices.mul(pPred, ct), Matrices.inverse(s));
            double[][] pNext = Matrices.mul(Matrices.sub(Matrices.identity(n), Matrices.mul(l, c)), pPred);
            double change = Matrices.maxAbs(Matrices.sub(pNext, p));
            p = pNext;
            if (change < 1e-14) return l;
        }
        throw new ArithmeticException("Kalman Riccati did not converge");
    }
}
