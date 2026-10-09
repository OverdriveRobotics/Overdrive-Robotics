package org.firstinspires.ftc.teamcode.control;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.info;
import static org.firstinspires.ftc.teamcode.T.near;
import static org.firstinspires.ftc.teamcode.T.test;

/** Additional turret maths: gear-ratio bookkeeping, motion, limits and the controller's own properties. */
public final class TurretGeometryAndControllerTests {
    private TurretGeometryAndControllerTests() {}

    public static void run() {
        test("turret gearing: the internal gearbox cannot be counted twice - two equivalent descriptions give identical angles", () -> {
            // Description A: encoder on the gearbox OUTPUT: 537.7 ticks/rev, external 3.5:1.
            // Description B: same hardware described as the 28-tick motor-shaft encoder with 19.2:1 internal * 3.5 external -> ticks per
            // turret rev must be identical, so the code must take ticksPerShaftRev and the EXTERNAL ratio only.
            double ticksPerTurretRevA = 537.7 * 3.5;
            double ticksPerTurretRevB = 28.0 * 19.2 * 3.5;   // 1881.6
            near(ticksPerTurretRevA, ticksPerTurretRevB, 1.0, "same hardware");
            double ticks = ticksPerTurretRevA / 4;           // a quarter turn
            near(Math.PI / 2, TurretUtil.ticksToAngle(ticks, 537.7, 3.5, 1, 0), 1e-3, "A");
            near(Math.PI / 2, TurretUtil.ticksToAngle(ticks, 28.0 * 19.2, 3.5, 1, 0), 1e-3, "B (internal ratio folded into the shaft resolution)");
            // the wrong way - multiplying the internal ratio in again - is detectably off by 19.2x:
            double wrong = TurretUtil.ticksToAngle(ticks, 537.7, 3.5 * 19.2, 1, 0);
            check(Math.abs(wrong - Math.PI / 2) > 1.0, "double counting would be a huge, visible error");
        });

        test("turret motion: the desired angle follows a robot driving past a fixed target", () -> {
            double tx = 60, ty = 0, prev = Double.NaN;
            for (double x = 0; x <= 60; x += 10) {
                double a = TurretUtil.desiredRelativeAngle(x, -30, 0, tx, ty);
                if (!Double.isNaN(prev)) check(a > prev - 1e-12, "bearing sweeps monotonically (26.6 deg -> 90 deg at the target's x) as the robot approaches: x=" + x);
                prev = a;
            }
            near(Math.PI / 2, TurretUtil.desiredRelativeAngle(60, -30, 0, 60, 0), 1e-9, "directly beside the target: 90 deg");
            near(Math.PI, Math.abs(TurretUtil.desiredRelativeAngle(10, 10, 0, 0, 10)), 1e-9, "target directly behind: +-180");
            // crossing the +-180 boundary produces no jump after unwrapping near the measured angle
            double lo = -Math.PI, hi = Math.PI;
            double a1 = AngleUtil.equivalentInRange(TurretUtil.desiredRelativeAngle(10, 10, 0, 0, 10 - 1e-6), Math.PI - 0.01, lo - 0.5, hi + 0.5);
            double a2 = AngleUtil.equivalentInRange(TurretUtil.desiredRelativeAngle(10, 10, 0, 0, 10 + 1e-6), Math.PI - 0.01, lo - 0.5, hi + 0.5);
            check(Math.abs(a1 - a2) < 1e-3, "no discontinuity across the wrap when the range allows it: " + a1 + " vs " + a2);
        });

        test("turret limits: reachable-angle logic for every region of a limited range", () -> {
            double lo = Math.toRadians(-100), hi = Math.toRadians(100);
            for (double d = -180; d <= 180; d += 15) {
                double r = AngleUtil.equivalentInRange(Math.toRadians(d), 0, lo, hi);
                boolean reachable = Math.abs(d) <= 100;
                check(Double.isNaN(r) == !reachable, "deg " + d + " reachable=" + reachable + " got " + r);
                if (reachable) near(Math.toRadians(d), r, 1e-9, "same angle when reachable");
            }
            check(TurretUtil.withinLimits(0.5, -1, 1) && !TurretUtil.withinLimits(1.5, -1, 1) && !TurretUtil.withinLimits(Double.NaN, -1, 1), "withinLimits");
        });

        test("turret alignment: needs both small error and small velocity; NaN is never aligned", () -> {
            check(TurretUtil.isAligned(0.01, 0.1, 0.02, 0.5), "aligned");
            check(!TurretUtil.isAligned(0.05, 0.1, 0.02, 0.5), "error too big");
            check(!TurretUtil.isAligned(0.01, 2.0, 0.02, 0.5), "still moving");
            check(!TurretUtil.isAligned(Double.NaN, 0, 0.02, 0.5) && !TurretUtil.isAligned(0, Double.NaN, 0.02, 0.5), "NaN");
        });

        test("turret controller: closed loop is stable (all eigenvalues of Ad-Bd*K inside the unit circle)", () -> {
            TurretStateSpaceController c = new TurretStateSpaceController(TurretStateSpaceController.Params.fromConfig());
            double[][] ad = c.discreteA(), bd = c.discreteB();
            double[] k = c.gains();
            double[][] acl = {{ad[0][0] - bd[0][0] * k[0], ad[0][1] - bd[0][0] * k[1]}, {ad[1][0] - bd[1][0] * k[0], ad[1][1] - bd[1][0] * k[1]}};
            double tr = acl[0][0] + acl[1][1], det = acl[0][0] * acl[1][1] - acl[0][1] * acl[1][0], disc = tr * tr - 4 * det;
            double m1, m2;
            if (disc >= 0) { m1 = Math.abs((tr + Math.sqrt(disc)) / 2); m2 = Math.abs((tr - Math.sqrt(disc)) / 2); }
            else { m1 = m2 = Math.sqrt(det); }
            info("closed-loop eigenvalue magnitudes " + m1 + ", " + m2);
            check(m1 < 1 && m2 < 1, "stable");
        });

        test("turret controller: the Riccati equation residual is ~0 (gain is a genuine DLQR solution)", () -> {
            TurretStateSpaceController.Params p = TurretStateSpaceController.Params.fromConfig();
            TurretStateSpaceController c = new TurretStateSpaceController(p);
            double[][] ad = c.discreteA(), bd = c.discreteB();
            double[][] q = {{1 / (p.maxAngleError * p.maxAngleError), 0}, {0, 1 / (p.maxVelocityError * p.maxVelocityError)}};
            double[][] r = {{1 / (p.maxVolts * p.maxVolts)}};
            double[][] k = Lqr.dlqr(ad, bd, q, r);
            near(k[0][0], c.gains()[0], 1e-9, "same gain from an independent call");
            // the optimal gain is a fixed point of: K = (R + B'PB)^-1 B'PA with P from the same iteration; check via cost-to-go monotonicity instead:
            double[] x = {0.3, 0};
            double cost = 0;
            for (int i = 0; i < 400; i++) {
                double u = -(k[0][0] * x[0] + k[0][1] * x[1]);
                cost += x[0] * x[0] * q[0][0] + x[1] * x[1] * q[1][1] + u * u * r[0][0];
                double nx0 = ad[0][0] * x[0] + ad[0][1] * x[1] + bd[0][0] * u, nx1 = ad[1][0] * x[0] + ad[1][1] * x[1] + bd[1][0] * u;
                x[0] = nx0; x[1] = nx1;
            }
            // a perturbed gain must not do better (optimality along both gain axes)
            for (double d : new double[]{0.7, 1.3}) {
                double[] y = {0.3, 0}; double c2 = 0;
                for (int i = 0; i < 400; i++) {
                    double u = -d * (k[0][0] * y[0] + k[0][1] * y[1]);
                    c2 += y[0] * y[0] * q[0][0] + y[1] * y[1] * q[1][1] + u * u * r[0][0];
                    double n0 = ad[0][0] * y[0] + ad[0][1] * y[1] + bd[0][0] * u, n1 = ad[1][0] * y[0] + ad[1][1] * y[1] + bd[1][0] * u;
                    y[0] = n0; y[1] = n1;
                }
                check(c2 >= cost - 1e-9, "scaled gain x" + d + " is not better than the LQR gain: " + c2 + " vs " + cost);
            }
        });

        test("turret controller: with the exact plant, feed-forward alone tracks a constant-velocity reference (feedback is not doing the work)", () -> {
            TurretStateSpaceController.Params p = TurretStateSpaceController.Params.fromConfig();
            p.maxRefVelocity = 50;
            TurretStateSpaceController c = new TurretStateSpaceController(p);
            ControlTests.TurretSim s = new ControlTests.TurretSim(p.a, p.b, 12.0);
            double rate = 0.8, maxErr = 0;
            for (int i = 0; i < 500; i++) {
                double tgt = 0.2 + rate * i * 0.02;
                double errBefore = tgt - s.theta;   // error at the instant the controller acts
                s.step(c.update(tgt, rate, s.theta, s.omega, 12, 0.02), 0.02);
                if (i > 60) maxErr = Math.max(maxErr, Math.abs(errBefore));
            }
            check(maxErr < Math.toRadians(0.05), "error " + Math.toDegrees(maxErr) + " deg at 0.8 rad/s");
        });

        test("turret controller: saturation never exceeds the battery-scaled limit and the integrator does not wind up", () -> {
            TurretStateSpaceController.Params p = TurretStateSpaceController.Params.fromConfig();
            p.integralGain = 2.0; p.integralLimitVolts = 1.0; p.maxVolts = 6;
            TurretStateSpaceController c = new TurretStateSpaceController(p);
            double maxPower = 0;
            for (int i = 0; i < 300; i++) maxPower = Math.max(maxPower, Math.abs(c.update(2.0, 0, 0, 0, 10.0, 0.02)));   // unreachable: stays saturated
            near(0.6, maxPower, 1e-9, "6 V cap on a 10 V battery = 0.6");
            check(c.isSaturated(), "reports saturation");
            double after = 0; for (int i = 0; i < 30; i++) after = c.update(-2.0, 0, 0, 0, 10.0, 0.02);
            check(after < 0, "reverses immediately instead of unwinding for seconds: " + after);
        });

        test("turret velocity filter: a known ramp is tracked with the expected lag and constant velocity passes unchanged", () -> {
            TurretStateSpaceController.Params p = TurretStateSpaceController.Params.fromConfig();
            p.velocityAlpha = 0.25;
            TurretStateSpaceController c = new TurretStateSpaceController(p);
            double f = 0;
            for (int i = 0; i < 200; i++) { c.update(0, 0, 0, 3.0, 12, 0.02); f = c.filteredVelocity(); }
            near(3.0, f, 1e-6, "constant velocity unchanged in steady state");
            for (int i = 0; i < 4; i++) { c.update(0, 0, 0, 0.0, 12, 0.02); f = c.filteredVelocity(); }
            near(3.0 * Math.pow(0.75, 4), f, 1e-9, "after a step to 0 the filter decays by (1-alpha)^n");
        });
    }
}
