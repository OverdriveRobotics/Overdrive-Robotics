package org.firstinspires.ftc.teamcode.control;

/**
 * Flywheel velocity controller: scalar state-space model + steady-state Kalman observer + LQR feedback +
 * plant-inversion feedforward (the structure of the WPILib state-space flywheel tutorial in data/datax.txt).
 *
 *   x = [omega]  (ticks/s),  u = motor voltage,  omega' = -a*omega + b*V
 *   x[k+1] = Ad x[k] + Bd u[k]      Ad = e^(-a T),  Bd = b (1 - e^(-a T)) / a
 *   u = (r - Ad r)/Bd  +  K (r - xhat)  (+ kS)       K from discrete LQR, xhat from the Kalman filter
 *
 * HONEST NOTE: with one state, LQR yields a proportional gain, so this is "model feedforward + P" whose gain
 * is chosen systematically from Q and R. It beats a hand-tuned PID mainly through the model feedforward and
 * the lag-free observer; it does NOT beat a well-tuned feedforward + P on the same plant (see
 * FlywheelControlTest for the simulated comparison). Its advantage is that gains follow from physical
 * parameters, and battery sag is handled because the output is VOLTS converted to power with the live battery
 * voltage.
 *
 * Gains K and L are designed once for the nominal period; the plant discretisation Ad/Bd is recomputed
 * per call from the measured dt (a scalar closed form, cheap).
 */
public final class FlywheelLqrController {

    public static final class Params {
        public double a, b;                    // omega' = -a*omega + b*V   (ticks/s, volts)
        public double kS = FlywheelConfig.KS_VOLTS;
        public double samplePeriod = FlywheelConfig.SAMPLE_PERIOD_S;
        public double maxVelocityError = FlywheelConfig.MAX_VELOCITY_ERROR;
        public double maxVolts = FlywheelConfig.MAX_CONTROL_VOLTS;
        public double modelStd = FlywheelConfig.MODEL_STD;
        public double measurementStd = FlywheelConfig.MEASUREMENT_STD;

        /** a, b from the PHYSICAL motor/inertia numbers in {@link FlywheelConfig}. */
        public static Params physical() {
            Params p = new Params();
            double g = FlywheelConfig.SHAFT_REVS_PER_FLYWHEEL_REV;
            double r = FlywheelConfig.NOMINAL_VOLTAGE / FlywheelConfig.STALL_CURRENT_A;
            double kt = FlywheelConfig.SHAFT_STALL_TORQUE_NM / FlywheelConfig.STALL_CURRENT_A;
            double kv = FlywheelConfig.NOMINAL_VOLTAGE / FlywheelConfig.SHAFT_FREE_SPEED_RAD_S;
            double jShaft = FlywheelConfig.FLYWHEEL_INERTIA_KG_M2 / (g * g);       // inertia reflected to the shaft
            double bShaft = FlywheelConfig.VISCOUS_FRICTION / (g * g);
            double n = FlywheelConfig.MOTOR_COUNT;
            double radToTicks = FlywheelConfig.SHAFT_TICKS_PER_REV / (2 * Math.PI);
            p.a = (n * kt * kv / r + bShaft) / jShaft;
            p.b = (n * kt / (r * jShaft)) * radToTicks;
            return p;
        }

        /** a, b from system identification: V = kS + kV*omega + kA*omega'. */
        public static Params sysId() {
            Params p = new Params();
            p.a = FlywheelConfig.KV_VOLTS_PER_TICK_S / FlywheelConfig.KA_VOLTS_PER_TICK_S2;
            p.b = 1.0 / FlywheelConfig.KA_VOLTS_PER_TICK_S2;
            return p;
        }

        public static Params fromConfig() {
            return FlywheelConfig.USE_SYSID_MODEL ? sysId() : physical();
        }
    }

    private final Params p;
    private final double k, l;
    private double xhat, uPrevVolts;
    private boolean initialised, saturated, fault;
    private double lastVolts, lastPower;

    public FlywheelLqrController(Params params) {
        if (!(params.a > 0) || !(params.b > 0) || !(params.samplePeriod > 0) || !(params.maxVolts > 0)
                || !(params.maxVelocityError > 0) || !(params.modelStd > 0) || !(params.measurementStd > 0)
                || !Double.isFinite(params.a) || !Double.isFinite(params.b) || !(params.kS >= 0)) {
            throw new IllegalArgumentException("invalid flywheel controller parameters");
        }
        this.p = params;
        double[][] ad = {{Math.exp(-p.a * p.samplePeriod)}};
        double[][] bd = {{p.b * (1 - ad[0][0]) / p.a}};
        k = Lqr.dlqr(ad, bd, new double[][]{{1 / sq(p.maxVelocityError)}}, new double[][]{{1 / sq(p.maxVolts)}})[0][0];
        l = Lqr.steadyStateKalman(ad, new double[][]{{1}}, new double[][]{{sq(p.modelStd)}},
                new double[][]{{sq(p.measurementStd)}})[0][0];
    }

    private static double sq(double v) { return v * v; }

    public double feedbackGain() { return k; }
    public double observerGain() { return l; }
    public double estimate() { return xhat; }

    public void reset() {
        initialised = false;
        saturated = false;
        fault = false;
        uPrevVolts = 0;
        lastVolts = lastPower = 0;
    }

    /**
     * @param target   reference velocity (ticks/s); &lt;= 0 means "off" and yields 0 power
     * @param measured motor velocity (ticks/s); NaN/inf is a fault and yields 0 power
     * @return power in [0, 1] (a flywheel is never driven in reverse here)
     */
    public double update(double target, double measured, double batteryVolts, double dt) {
        if (!Double.isFinite(measured) || !Double.isFinite(target) || !(batteryVolts > 1.0)
                || !Double.isFinite(batteryVolts) || !(dt > 0) || !Double.isFinite(dt)
                || Math.abs(measured) > FlywheelConfig.MAX_PLAUSIBLE_VELOCITY * 4) {
            fault = true;
            initialised = false;
            lastVolts = lastPower = 0;
            uPrevVolts = 0;
            saturated = false;
            return 0;
        }
        fault = false;
        if (target <= 0) {
            xhat = measured;
            initialised = true;
            uPrevVolts = lastVolts = lastPower = 0;
            saturated = false;
            return 0;
        }
        double ad = Math.exp(-p.a * dt), bd = p.b * (1 - ad) / p.a;
        if (!initialised) {
            xhat = measured;
            initialised = true;
        } else {
            double vEff = Math.max(0, uPrevVolts - p.kS);
            double pred = ad * xhat + bd * vEff;
            xhat = pred + l * (measured - pred);
        }
        double uff = (target - ad * target) / bd + p.kS;
        double u = uff + k * (target - xhat);
        double limit = Math.min(p.maxVolts, batteryVolts);
        double uSat = AngleUtil.clamp(u, 0, limit);
        saturated = uSat != u;
        uPrevVolts = uSat;
        lastVolts = uSat;
        lastPower = uSat / batteryVolts;
        return lastPower;
    }

    public boolean isSaturated() { return saturated; }
    public boolean hasFault() { return fault; }
    public double lastVolts() { return lastVolts; }
    public double lastPower() { return lastPower; }

    /** Hand-tuned-style baseline for comparison: kV/kA feedforward plus a fixed P gain, no observer. */
    public static final class Baseline {
        private final double kV, kS, kP, maxVolts;
        public Baseline(double kV, double kS, double kP, double maxVolts) {
            this.kV = kV; this.kS = kS; this.kP = kP; this.maxVolts = maxVolts;
        }
        public double update(double target, double measured, double batteryVolts) {
            if (target <= 0 || !Double.isFinite(measured)) return 0;
            double u = kS + kV * target + kP * (target - measured);
            return AngleUtil.clamp(u, 0, Math.min(maxVolts, batteryVolts)) / batteryVolts;
        }
    }
}
