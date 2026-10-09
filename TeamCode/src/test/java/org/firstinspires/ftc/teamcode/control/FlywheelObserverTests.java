package org.firstinspires.ftc.teamcode.control;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.info;
import static org.firstinspires.ftc.teamcode.T.near;
import static org.firstinspires.ftc.teamcode.T.test;

import java.util.Random;

/** Flywheel observer, units, feed-forward and fault handling with deterministic synthetic data. */
public final class FlywheelObserverTests {
    private FlywheelObserverTests() {}

    public static void run() {
        test("flywheel units: the plant gain scales with encoder resolution; a rad/s model and a ticks/s model agree", () -> {
            FlywheelLqrController.Params p1 = FlywheelLqrController.Params.physical();
            double tpr = FlywheelConfig.SHAFT_TICKS_PER_REV;
            FlywheelConfig.SHAFT_TICKS_PER_REV = 2 * tpr;
            FlywheelLqrController.Params p2 = FlywheelLqrController.Params.physical();
            FlywheelConfig.SHAFT_TICKS_PER_REV = tpr;
            near(p1.a, p2.a, 1e-12, "time constant independent of the encoder");
            near(2 * p1.b, p2.b, 1e-9, "b (ticks/s per volt) doubles with ticks per rev");
            // steady state: omega_rad = b_rad*V/a with b_rad = b*2pi/tpr must not depend on the encoder
            near(p1.b * 2 * Math.PI / tpr / p1.a, p2.b * 2 * Math.PI / (2 * tpr) / p2.a, 1e-9, "steady-state rad/s per volt is encoder independent");
        });

        test("flywheel units: sysid and physical descriptions of the same plant give the same controller gains", () -> {
            FlywheelLqrController.Params phys = FlywheelLqrController.Params.physical();
            FlywheelLqrController.Params sys = new FlywheelLqrController.Params();
            sys.a = phys.a; sys.b = phys.b;   // as if kV = a/b, kA = 1/b
            near(new FlywheelLqrController(phys).feedbackGain(), new FlywheelLqrController(sys).feedbackGain(), 1e-12, "K");
            double kv = phys.a / phys.b, ka = 1 / phys.b;
            FlywheelConfig.KV_VOLTS_PER_TICK_S = kv; FlywheelConfig.KA_VOLTS_PER_TICK_S2 = ka;
            FlywheelLqrController.Params fromSys = FlywheelLqrController.Params.sysId();
            near(phys.a, fromSys.a, 1e-9, "a"); near(phys.b, fromSys.b, 1e-6, "b");
            org.firstinspires.ftc.teamcode.tuning.ParamRegistry.resetToDefaults();
        });

        test("flywheel feed-forward: in steady state the commanded volts equal kV * velocity (+kS), with no feedback contribution", () -> {
            FlywheelLqrController.Params p = FlywheelLqrController.Params.physical();
            p.kS = 0.4;
            FlywheelLqrController c = new FlywheelLqrController(p);
            double target = 1500, v = 0;
            for (int i = 0; i < 400; i++) {
                double e = Math.exp(-p.a * 0.02);
                double u = c.update(target, target, 12, 0.02) * 12;   // measurement already at the target
                v = target;
                if (i > 100) near(p.kS + (p.a / p.b) * target, u, 1e-6, "volts at i=" + i);
            }
        });

        test("flywheel observer: converges from a wrong initial estimate and rejects measurement noise (RMS below the raw noise)", () -> {
            FlywheelLqrController.Params p = FlywheelLqrController.Params.physical();
            p.measurementStd = 40; p.modelStd = 20;
            FlywheelLqrController c = new FlywheelLqrController(p);
            Random rng = new Random(42);
            double a = p.a, b = p.b, w = 1800, dt = 0.02, sumRaw = 0, sumEst = 0; int n = 0;
            for (int i = 0; i < 1500; i++) {
                double meas = w + rng.nextGaussian() * 40;
                double u = c.update(1800, i == 0 ? 400 : meas, 12, dt);   // first reading is wildly wrong
                double e = Math.exp(-a * dt);
                w = e * w + b * (1 - e) / a * (u * 12);
                if (i > 200) { sumRaw += (meas - w) * (meas - w); sumEst += (c.estimate() - w) * (c.estimate() - w); n++; }
            }
            double rawRms = Math.sqrt(sumRaw / n), estRms = Math.sqrt(sumEst / n);
            info("raw noise rms " + rawRms + ", estimate error rms " + estRms);
            check(estRms < 0.8 * rawRms, "estimate is smoother than the raw measurement");
            check(Math.abs(c.estimate() - 1800) < 100, "converged near the true value");
        });

        test("flywheel observer: tracks a step disturbance (ball) quickly with a large model std, slowly with a small one", () -> {
            double[] lag = new double[2];
            double[] stds = {10, 400};
            for (int k = 0; k < 2; k++) {
                FlywheelLqrController.Params p = FlywheelLqrController.Params.physical();
                p.modelStd = stds[k]; p.measurementStd = 40;
                FlywheelLqrController c = new FlywheelLqrController(p);
                double w = 1800;
                for (int i = 0; i < 300; i++) { c.update(1800, w, 12, 0.02); }
                w = 1500;   // ball: instant 300 drop, held for the test
                int steps = 0;
                while (Math.abs(c.estimate() - 1500) > 30 && steps < 200) { c.update(1800, w, 12, 0.02); steps++; }
                lag[k] = steps;
            }
            info("steps to follow a 300 tick/s drop: model_std 10 -> " + lag[0] + ", 400 -> " + lag[1]);
            check(lag[1] < lag[0], "bigger model std reacts faster");
        });

        test("flywheel target changes: reference tracking reaches both a higher and a lower target within limits", () -> {
            FlywheelLqrController.Params p = FlywheelLqrController.Params.physical();
            FlywheelLqrController c = new FlywheelLqrController(p);
            ControlTests.FlywheelSim s = new ControlTests.FlywheelSim(p.a, p.b);
            double[] targets = {1000, 2400, 1500, 0};
            for (double t : targets) {
                for (int i = 0; i < 700; i++) { double u = c.update(t, s.omega, 12, 0.02); check(u >= 0 && u <= 1, "power range"); s.step(u, 0.02); }
                if (t > 0) near(t, s.omega, 8, "reached " + t);
            }
            check(s.omega < 5 || c.lastPower() == 0, "target 0 means off");
        });

        test("flywheel faults: NaN/inf/absurd readings and a dead battery sensor give zero output and then recover cleanly", () -> {
            FlywheelLqrController c = new FlywheelLqrController(FlywheelLqrController.Params.physical());
            for (double bad : new double[]{Double.NaN, Double.POSITIVE_INFINITY, 1e9}) {
                near(0, c.update(1800, bad, 12, 0.02), 0, "reading " + bad);
                check(c.hasFault(), "fault flagged for " + bad);
            }
            near(0, c.update(1800, 100, Double.NaN, 0.02), 0, "battery NaN");
            near(0, c.update(1800, 100, 12, 0), 0, "dt 0");
            double ok = c.update(1800, 100, 12, 0.02);
            check(!c.hasFault() && ok > 0, "recovers on the next valid sample (re-seeded observer)");
        });

        test("flywheel battery compensation: the same velocity at lower voltage needs proportionally more power", () -> {
            FlywheelLqrController.Params p = FlywheelLqrController.Params.physical();
            double[] pw = new double[2];
            double[] volts = {12, 9};
            for (int k = 0; k < 2; k++) {
                FlywheelLqrController c = new FlywheelLqrController(p);
                pw[k] = c.update(1800, 1800, volts[k], 0.02);
            }
            near(pw[0] * 12 / 9, pw[1], 1e-9, "power * volts is constant");
        });
    }
}
