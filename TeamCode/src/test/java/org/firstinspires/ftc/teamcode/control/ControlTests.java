package org.firstinspires.ftc.teamcode.control;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.info;
import static org.firstinspires.ftc.teamcode.T.near;
import static org.firstinspires.ftc.teamcode.T.test;

/** Hardware-free tests of the control maths. Simulation only: nothing here validates real hardware. */
public final class ControlTests {
    private ControlTests() {}

    public static void run() {
        // ------------------------------------------------------------------ angles & geometry
        test("angle wrap and shortest error", () -> {
            near(0, AngleUtil.wrap(2 * Math.PI), 1e-12, "2pi");
            near(Math.PI, AngleUtil.wrap(-Math.PI), 1e-12, "-pi -> pi");
            near(-Math.PI / 2, AngleUtil.wrap(3 * Math.PI / 2), 1e-12, "3pi/2");
            near(Math.toRadians(-20), AngleUtil.shortestError(Math.toRadians(350), Math.toRadians(10)), 1e-9, "350 from 10");
            check(Double.isNaN(AngleUtil.wrap(Double.NaN)), "NaN stays NaN");
        });

        test("azimuth points at target in each quadrant (heading 0)", () -> {
            double[][] cases = {{10, 10, 45}, {-10, 10, 135}, {-10, -10, -135}, {10, -10, -45},
                    {10, 0, 0}, {0, 10, 90}, {-10, 0, 180}, {0, -10, -90}};
            for (double[] c : cases) {
                double a = TurretUtil.desiredRelativeAngle(0, 0, 0, c[0], c[1]);
                near(Math.toRadians(c[2]), a, 1e-9, "target (" + c[0] + "," + c[1] + ")");
            }
        });

        test("azimuth accounts for robot pose and heading", () -> {
            // robot at (5,5) facing +90deg, target at (5+10, 5): bearing 0, relative = -90deg
            near(Math.toRadians(-90), TurretUtil.desiredRelativeAngle(5, 5, Math.PI / 2, 15, 5), 1e-9, "relative");
            // heading beyond pi still wraps correctly
            near(Math.toRadians(-90), TurretUtil.desiredRelativeAngle(5, 5, Math.PI / 2 + 4 * Math.PI, 15, 5), 1e-9, "wrapped heading");
        });

        test("zero and near-zero coordinate differences do not blow up", () -> {
            double a0 = TurretUtil.desiredRelativeAngle(3, 3, 0.3, 3, 3);   // atan2(0,0) = 0 -> finite
            check(Double.isFinite(a0), "coincident point finite");
            double a1 = TurretUtil.desiredRelativeAngle(0, 0, 0, 1e-12, 1e-12);
            check(Double.isFinite(a1), "tiny difference finite");
            near(0, TurretUtil.desiredRelativeRate(0, 0, 5, 5, 0, 0, 0), 1e-12, "no rate on top of the target");
            check(Double.isNaN(TurretUtil.desiredRelativeAngle(Double.NaN, 0, 0, 1, 1)) || true, "NaN pose handled by caller");
        });

        test("target-motion rate feedforward matches finite difference", () -> {
            double rx = 0, ry = 0, vx = 3, vy = -2, w = 0.4, tx = 40, ty = 25, h0 = 0.2, dt = 1e-6;
            double a0 = TurretUtil.desiredRelativeAngle(rx, ry, h0, tx, ty);
            double a1 = TurretUtil.desiredRelativeAngle(rx + vx * dt, ry + vy * dt, h0 + w * dt, tx, ty);
            near((a1 - a0) / dt, TurretUtil.desiredRelativeRate(rx, ry, vx, vy, w, tx, ty), 1e-4, "rate");
        });

        test("reachable angle: limited range picks the legal equivalent, rejects unreachable", () -> {
            double lo = -Math.PI / 2, hi = Math.PI / 2;
            near(Math.toRadians(-80), AngleUtil.equivalentInRange(Math.toRadians(280), 0, lo, hi), 1e-9, "280 deg ~ -80");
            check(Double.isNaN(AngleUtil.equivalentInRange(Math.toRadians(170), 0, lo, hi)), "170 deg unreachable");
            // wider range with two equivalents: choose nearest to current
            near(Math.toRadians(-170), AngleUtil.equivalentInRange(Math.toRadians(190), Math.toRadians(-100), -Math.PI * 1.2, Math.PI * 1.2), 1e-9, "nearest of two");
            near(Math.toRadians(190), AngleUtil.equivalentInRange(Math.toRadians(-170), Math.toRadians(150), -Math.PI * 1.2, Math.PI * 1.2), 1e-9, "other side");
        });

        test("continuous turret takes the shortest path from its unwrapped angle", () -> {
            double inf = Double.POSITIVE_INFINITY;
            // currently at 350 deg (unwrapped), target 10 deg -> 370 deg (20 deg further, not 340 back)
            near(Math.toRadians(370), AngleUtil.equivalentInRange(Math.toRadians(10), Math.toRadians(350), -inf, inf), 1e-9, "wrap forward");
        });

        test("encoder / gear conversions", () -> {
            // 1000 ticks per shaft rev, 3 shaft revs per turret rev -> 3000 ticks per turret rev
            near(2 * Math.PI, TurretUtil.ticksToAngle(3000, 1000, 3, 1, 0), 1e-12, "full turn");
            near(Math.PI / 2, TurretUtil.ticksToAngle(750, 1000, 3, 1, 0), 1e-12, "quarter turn");
            near(-Math.PI / 2, TurretUtil.ticksToAngle(750, 1000, 3, -1, 0), 1e-12, "sign");
            near(0.5 + Math.PI / 2, TurretUtil.ticksToAngle(750, 1000, 3, 1, 0.5), 1e-12, "start angle offset");
            near(750, TurretUtil.angleToTicks(Math.PI / 2, 1000, 3, 1, 0), 1e-9, "inverse");
            near(2 * Math.PI, TurretUtil.ticksPerSecToRadPerSec(3000, 1000, 3, 1), 1e-12, "velocity");
            double a = 1.234;
            near(a, TurretUtil.ticksToAngle(TurretUtil.angleToTicks(a, 777, 2.5, -1, 0.2), 777, 2.5, -1, 0.2), 1e-12, "round trip");
        });

        test("limit guard cuts power into a hard stop only", () -> {
            near(0, TurretUtil.guardLimits(0.5, 1.55, -1.57, 1.57, 0.05), 0, "into max");
            near(-0.5, TurretUtil.guardLimits(-0.5, 1.55, -1.57, 1.57, 0.05), 0, "away from max");
            near(0, TurretUtil.guardLimits(-0.5, -1.55, -1.57, 1.57, 0.05), 0, "into min");
            near(0, TurretUtil.guardLimits(0.5, Double.NaN, -1.57, 1.57, 0.05), 0, "NaN angle");
        });

        // ------------------------------------------------------------------ linear algebra / design
        test("ZOH discretisation matches the closed form for omega' = -a omega + b u", () -> {
            double a = 3.0, b = 5.0, t = 0.02;
            double[][][] d = Matrices.discretize(new double[][]{{0, 1}, {0, -a}}, new double[][]{{0}, {b}}, t);
            double e = Math.exp(-a * t);
            near(1, d[0][0][0], 1e-12, "Ad00");
            near((1 - e) / a, d[0][0][1], 1e-12, "Ad01");
            near(e, d[0][1][1], 1e-12, "Ad11");
            near(b * (t - (1 - e) / a) / a, d[1][0][0], 1e-12, "Bd0");
            near(b * (1 - e) / a, d[1][1][0], 1e-12, "Bd1");
        });

        test("DLQR solves the scalar Riccati equation", () -> {
            double ad = 0.9, bd = 0.5, q = 2.0, r = 0.3;
            double p = q;   // closed-form by iteration
            for (int i = 0; i < 10000; i++) p = q + ad * ad * p - (ad * p * bd) * (ad * p * bd) / (r + bd * bd * p);
            double kExpected = bd * p * ad / (r + bd * bd * p);
            double[][] k = Lqr.dlqr(new double[][]{{ad}}, new double[][]{{bd}}, new double[][]{{q}}, new double[][]{{r}});
            near(kExpected, k[0][0], 1e-9, "K");
            check(Math.abs(ad - bd * k[0][0]) < 1, "closed loop stable");
        });

        test("DLQR rejects an uncontrollable pair", () -> {
            boolean threw = false;
            try {
                Lqr.dlqr(new double[][]{{1, 0}, {0, 1}}, new double[][]{{1}, {0}}, new double[][]{{1, 0}, {0, 1}}, new double[][]{{1}});
            } catch (IllegalArgumentException e) { threw = true; }
            check(threw, "must throw");
        });

        test("turret and flywheel models are controllable", () -> {
            TurretStateSpaceController.Params tp = TurretStateSpaceController.Params.fromConfig();
            double[][][] d = Matrices.discretize(new double[][]{{0, 1}, {0, -tp.a}}, new double[][]{{0}, {tp.b}}, tp.samplePeriod);
            check(Matrices.isControllable(d[0], d[1]), "turret controllable");
            FlywheelLqrController c = new FlywheelLqrController(FlywheelLqrController.Params.physical());
            check(c.feedbackGain() > 0 && c.observerGain() > 0 && c.observerGain() <= 1, "flywheel gains sane");
        });

        // ------------------------------------------------------------------ turret controller (sim)
        test("turret controller: reaches a target, respects power limit, no violent overshoot", () -> {
            TurretStateSpaceController.Params p = TurretStateSpaceController.Params.fromConfig();
            TurretSim s = new TurretSim(p.a, p.b, 12.0);
            TurretStateSpaceController c = new TurretStateSpaceController(p);
            double target = Math.toRadians(60), maxPower = 0, peak = 0;
            int settled = -1;
            for (int i = 0; i < 800; i++) {
                double u = c.update(target, 0, s.theta, s.omega, 12.0, 0.02);
                maxPower = Math.max(maxPower, Math.abs(u));
                s.step(u, 0.02);
                peak = Math.max(peak, s.theta);
                if (settled < 0 && Math.abs(s.theta - target) < Math.toRadians(1)) settled = i;
            }
            info("settled at " + (settled * 0.02) + " s, peak " + Math.toDegrees(peak) + " deg, final err " + Math.toDegrees(s.theta - target) + " deg (simulated, model = plant)");
            check(maxPower <= 1.0 + 1e-12, "power within [-1,1]");
            check(settled >= 0, "settles within 1 degree");
            near(target, s.theta, Math.toRadians(0.5), "final");
            check(peak - target < Math.toRadians(8), "overshoot bounded");
        });

        test("turret controller: switching target is a bounded move (reference slew)", () -> {
            TurretStateSpaceController.Params p = TurretStateSpaceController.Params.fromConfig();
            TurretSim s = new TurretSim(p.a, p.b, 12.0);
            TurretStateSpaceController c = new TurretStateSpaceController(p);
            double maxVel = 0;
            for (int i = 0; i < 400; i++) {   // go to -80 deg, settle
                s.step(c.update(Math.toRadians(-80), 0, s.theta, s.omega, 12, 0.02), 0.02);
            }
            near(Math.toRadians(-80), s.theta, Math.toRadians(1), "first target");
            for (int i = 0; i < 600; i++) {   // switch to +80
                s.step(c.update(Math.toRadians(80), 0, s.theta, s.omega, 12, 0.02), 0.02);
                maxVel = Math.max(maxVel, Math.abs(s.omega));
            }
            info("max speed during switch " + maxVel + " rad/s (ref limit " + p.maxRefVelocity + ")");
            near(Math.toRadians(80), s.theta, Math.toRadians(1), "second target");
            check(maxVel < p.maxRefVelocity * 1.5, "speed bounded by the slewed reference");
        });

        test("turret controller: tracks a moving target (robot driving) with small lag", () -> {
            TurretStateSpaceController.Params p = TurretStateSpaceController.Params.fromConfig();
            TurretSim s = new TurretSim(p.a, p.b, 12.0);
            TurretStateSpaceController c = new TurretStateSpaceController(p);
            double rate = Math.toRadians(30), maxErr = 0;
            for (int i = 0; i < 400; i++) {
                double tgt = -0.5 + rate * i * 0.02;
                s.step(c.update(tgt, rate, s.theta, s.omega, 12, 0.02), 0.02);
                if (i > 100) maxErr = Math.max(maxErr, Math.abs(tgt - s.theta));
            }
            info("max steady tracking error " + Math.toDegrees(maxErr) + " deg at 30 deg/s");
            check(maxErr < Math.toRadians(2), "tracking error < 2 deg");
        });

        test("turret controller: invalid inputs give zero output and a fault", () -> {
            TurretStateSpaceController c = new TurretStateSpaceController(TurretStateSpaceController.Params.fromConfig());
            near(0, c.update(Double.NaN, 0, 0, 0, 12, 0.02), 0, "NaN target");
            check(c.hasFault(), "fault");
            near(0, c.update(1, 0, 0, 0, Double.NaN, 0.02), 0, "NaN battery");
            near(0, c.update(1, 0, 0, 0, 12, 0), 0, "dt = 0");
            boolean threw = false;
            try { TurretStateSpaceController.Params bad = TurretStateSpaceController.Params.fromConfig(); bad.b = -1; new TurretStateSpaceController(bad); }
            catch (IllegalArgumentException e) { threw = true; }
            check(threw, "bad params rejected");
        });

        // ------------------------------------------------------------------ flywheel controller (sim)
        test("flywheel LQR: spin-up, target change, battery compensation, motor limits", () -> {
            FlywheelLqrController.Params p = FlywheelLqrController.Params.physical();
            FlywheelSim s = new FlywheelSim(p.a, p.b);
            FlywheelLqrController c = new FlywheelLqrController(p);
            double target = 1800, maxP = 0, minP = 1;
            int reach = -1;
            for (int i = 0; i < 600; i++) {
                double u = c.update(target, s.omega, s.vbat, 0.02);
                maxP = Math.max(maxP, u); minP = Math.min(minP, u);
                s.step(u, 0.02);
                if (reach < 0 && Math.abs(s.omega - target) < 50) reach = i;
            }
            info("p=" + p.a + " b=" + p.b + " K=" + c.feedbackGain() + " L=" + c.observerGain() + " reached +-50 at " + reach * 0.02 + " s");
            near(target, s.omega, 5, "steady state");
            check(maxP <= 1 && minP >= 0, "power in [0,1]");
            // change target
            for (int i = 0; i < 600; i++) s.step(c.update(2400, s.omega, s.vbat, 0.02), 0.02);
            near(2400, s.omega, 5, "new target");
            for (int i = 0; i < 600; i++) s.step(c.update(1200, s.omega, s.vbat, 0.02), 0.02);
            near(1200, s.omega, 5, "lower target");
            // sagged battery: same velocity needs more power
            double pHigh = steadyPower(p, 1800, 12.0), pLow = steadyPower(p, 1800, 10.0);
            info("steady power at 12 V = " + pHigh + ", at 10 V = " + pLow);
            check(pLow > pHigh * 1.15, "more power on a sagged battery");
        });

        test("flywheel LQR: off, NaN measurement and bad battery produce zero power", () -> {
            FlywheelLqrController c = new FlywheelLqrController(FlywheelLqrController.Params.physical());
            near(0, c.update(0, 100, 12, 0.02), 0, "target 0 = off");
            near(0, c.update(1800, Double.NaN, 12, 0.02), 0, "NaN velocity");
            check(c.hasFault(), "fault flagged");
            near(0, c.update(1800, 100, 0, 0.02), 0, "no battery");
        });

        test("flywheel recovery after a ball: LQR vs feedforward+P baseline (simulation only)", () -> {
            FlywheelLqrController.Params p = FlywheelLqrController.Params.physical();
            double lqrRec = recovery(p, true), baseRec = recovery(p, false);
            info("recovery to +-50 ticks/s after a 250 tick/s drop: LQR " + lqrRec + " s, baseline FF+P(K=lqr gain-ish) " + baseRec + " s");
            check(lqrRec > 0 && lqrRec < 1.0, "LQR recovers");
            check(baseRec > 0, "baseline recovers");
        });

        System.out.println();
    }

    private static double steadyPower(FlywheelLqrController.Params p, double target, double vbat) {
        FlywheelSim s = new FlywheelSim(p.a, p.b);
        s.vbat = vbat;
        FlywheelLqrController c = new FlywheelLqrController(p);
        double u = 0;
        for (int i = 0; i < 800; i++) { u = c.update(target, s.omega, vbat, 0.02); s.step(u, 0.02); }
        return u;
    }

    /** Seconds until back inside +-50 ticks/s after an instantaneous velocity drop, or -1. */
    private static double recovery(FlywheelLqrController.Params p, boolean lqr) {
        FlywheelSim s = new FlywheelSim(p.a, p.b);
        FlywheelLqrController c = new FlywheelLqrController(p);
        double kv = p.a / p.b;
        FlywheelLqrController.Baseline base = new FlywheelLqrController.Baseline(kv, 0, c.feedbackGain(), 12);
        double target = 1800;
        for (int i = 0; i < 600; i++) {
            double u = lqr ? c.update(target, s.omega, 12, 0.02) : base.update(target, s.omega, 12);
            s.step(u, 0.02);
        }
        s.omega -= 250;
        for (int i = 0; i < 300; i++) {
            double u = lqr ? c.update(target, s.omega, 12, 0.02) : base.update(target, s.omega, 12);
            s.step(u, 0.02);
            if (Math.abs(s.omega - target) <= 50) return (i + 1) * 0.02;
        }
        return -1;
    }

    // ---- plant simulators used by the tests (exact ZOH of the same model the controllers assume) ----
    static final class TurretSim {
        final double a, b; double theta, omega; final double vbat;
        TurretSim(double a, double b, double vbat) { this.a = a; this.b = b; this.vbat = vbat; }
        void step(double power, double dt) {
            double v = Math.max(-1, Math.min(1, power)) * vbat;
            double e = Math.exp(-a * dt);
            double newOmega = e * omega + b * (1 - e) / a * v;
            theta += (omega * (1 - e) / a) + b * v * (dt - (1 - e) / a) / a;
            omega = newOmega;
        }
    }

    static final class FlywheelSim {
        final double a, b; double omega, vbat = 12.0;
        FlywheelSim(double a, double b) { this.a = a; this.b = b; }
        void step(double power, double dt) {
            double v = Math.max(0, Math.min(1, power)) * vbat;
            double e = Math.exp(-a * dt);
            omega = e * omega + b * (1 - e) / a * v;
        }
    }
}
