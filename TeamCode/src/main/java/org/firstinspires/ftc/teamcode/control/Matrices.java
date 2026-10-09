package org.firstinspires.ftc.teamcode.control;

/**
 * Minimal dense-matrix helpers for the small (n &lt;= 4) models used here. Only used at construction time to
 * discretise models and solve Riccati equations; the per-loop controller code uses plain scalars.
 */
public final class Matrices {
    private Matrices() {}

    public static double[][] zeros(int r, int c) { return new double[r][c]; }

    public static double[][] identity(int n) {
        double[][] m = new double[n][n];
        for (int i = 0; i < n; i++) m[i][i] = 1;
        return m;
    }

    public static double[][] copy(double[][] a) {
        double[][] m = new double[a.length][];
        for (int i = 0; i < a.length; i++) m[i] = a[i].clone();
        return m;
    }

    public static double[][] mul(double[][] a, double[][] b) {
        int n = a.length, k = b.length, m = b[0].length;
        if (a[0].length != k) throw new IllegalArgumentException("dimension mismatch");
        double[][] r = new double[n][m];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++) {
                double s = 0;
                for (int t = 0; t < k; t++) s += a[i][t] * b[t][j];
                r[i][j] = s;
            }
        return r;
    }

    public static double[][] add(double[][] a, double[][] b) { return combine(a, b, 1); }
    public static double[][] sub(double[][] a, double[][] b) { return combine(a, b, -1); }

    private static double[][] combine(double[][] a, double[][] b, double sign) {
        double[][] r = new double[a.length][a[0].length];
        for (int i = 0; i < a.length; i++)
            for (int j = 0; j < a[0].length; j++) r[i][j] = a[i][j] + sign * b[i][j];
        return r;
    }

    public static double[][] scale(double[][] a, double s) {
        double[][] r = new double[a.length][a[0].length];
        for (int i = 0; i < a.length; i++)
            for (int j = 0; j < a[0].length; j++) r[i][j] = a[i][j] * s;
        return r;
    }

    public static double[][] transpose(double[][] a) {
        double[][] r = new double[a[0].length][a.length];
        for (int i = 0; i < a.length; i++)
            for (int j = 0; j < a[0].length; j++) r[j][i] = a[i][j];
        return r;
    }

    public static double maxAbs(double[][] a) {
        double m = 0;
        for (double[] row : a) for (double v : row) m = Math.max(m, Math.abs(v));
        return m;
    }

    /** Gauss-Jordan inverse with partial pivoting. Throws if singular. */
    public static double[][] inverse(double[][] a) {
        int n = a.length;
        double[][] m = new double[n][2 * n];
        for (int i = 0; i < n; i++) {
            System.arraycopy(a[i], 0, m[i], 0, n);
            m[i][n + i] = 1;
        }
        for (int c = 0; c < n; c++) {
            int p = c;
            for (int r = c + 1; r < n; r++) if (Math.abs(m[r][c]) > Math.abs(m[p][c])) p = r;
            if (Math.abs(m[p][c]) < 1e-14) throw new ArithmeticException("singular matrix");
            double[] t = m[p]; m[p] = m[c]; m[c] = t;
            double d = m[c][c];
            for (int j = 0; j < 2 * n; j++) m[c][j] /= d;
            for (int r = 0; r < n; r++) {
                if (r == c) continue;
                double f = m[r][c];
                if (f == 0) continue;
                for (int j = 0; j < 2 * n; j++) m[r][j] -= f * m[c][j];
            }
        }
        double[][] inv = new double[n][n];
        for (int i = 0; i < n; i++) System.arraycopy(m[i], n, inv[i], 0, n);
        return inv;
    }

    /** Matrix exponential by scaling-and-squaring with a Taylor series. */
    public static double[][] expm(double[][] a) {
        int n = a.length;
        double norm = maxAbs(a) * n;
        int squarings = norm > 0.5 ? (int) Math.ceil(Math.log(norm / 0.5) / Math.log(2)) : 0;
        double[][] s = scale(a, 1.0 / Math.pow(2, squarings));
        double[][] result = identity(n), term = identity(n);
        for (int k = 1; k <= 20; k++) {
            term = scale(mul(term, s), 1.0 / k);
            result = add(result, term);
        }
        for (int i = 0; i < squarings; i++) result = mul(result, result);
        return result;
    }

    /** Zero-order-hold discretisation. Returns {Ad, Bd} for x' = A x + B u sampled every {@code t} seconds. */
    public static double[][][] discretize(double[][] a, double[][] b, double t) {
        int n = a.length, m = b[0].length;
        double[][] aug = new double[n + m][n + m];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) aug[i][j] = a[i][j] * t;
            for (int j = 0; j < m; j++) aug[i][n + j] = b[i][j] * t;
        }
        double[][] e = expm(aug);
        double[][] ad = new double[n][n], bd = new double[n][m];
        for (int i = 0; i < n; i++) {
            System.arraycopy(e[i], 0, ad[i], 0, n);
            for (int j = 0; j < m; j++) bd[i][j] = e[i][n + j];
        }
        return new double[][][]{ad, bd};
    }

    /** Numerical rank by Gaussian elimination. */
    public static int rank(double[][] a) {
        double[][] m = copy(a);
        int rows = m.length, cols = m[0].length, rank = 0;
        for (int c = 0; c < cols && rank < rows; c++) {
            int p = rank;
            for (int r = rank + 1; r < rows; r++) if (Math.abs(m[r][c]) > Math.abs(m[p][c])) p = r;
            if (Math.abs(m[p][c]) < 1e-10) continue;
            double[] t = m[p]; m[p] = m[rank]; m[rank] = t;
            for (int r = rank + 1; r < rows; r++) {
                double f = m[r][c] / m[rank][c];
                for (int j = c; j < cols; j++) m[r][j] -= f * m[rank][j];
            }
            rank++;
        }
        return rank;
    }

    /** True if (A, B) is controllable: rank [B AB ... A^(n-1)B] == n. */
    public static boolean isControllable(double[][] a, double[][] b) {
        int n = a.length, m = b[0].length;
        double[][] ctrb = new double[n][n * m];
        double[][] cur = b;
        for (int k = 0; k < n; k++) {
            for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) ctrb[i][k * m + j] = cur[i][j];
            cur = mul(a, cur);
        }
        return rank(ctrb) == n;
    }
}
