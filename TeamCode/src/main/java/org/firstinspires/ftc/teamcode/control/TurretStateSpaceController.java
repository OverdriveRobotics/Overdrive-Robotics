package org.firstinspires.ftc.teamcode.control;

/**
 * Turret state-space controller. Hardware-free: feed it measurements, read back a motor power.
 *
 * Plant (turret output side, input = motor voltage):
 *   x = [theta, omega],  theta' = omega,  omega' = -a*omega + b*V
 *   a = (G^2*Kt*Kv/R + bf)/J,  b = G*Kt/(R*J)        (G = shaft revs per turret rev, see {@link TurretConfig})
 * discretised with a zero-order hold at the nominal period T:  x[k+1] = Ad x[k] + Bd u[k],  y = [1 0] x.
 *
 * Control law (reference tracking, WPILib-style):
 *   u = Bd^+ (r[k+1] - Ad r[k])  +  K (r[k] - x[k])  +  kS*sign(...)  +  integral
 * with K from discrete LQR (Bryson-rule Q, R) and r = [slew-limited reference angle, reference velocity].
 * The reference velocity comes from the target-motion feedforward (robot moving relative to the target).
 *
 * Angle wrap is handled BEFORE this class: the caller passes a reachable (unwrapped) desired angle chosen
 * near the measured angle, so the position error here is a plain subtraction.
 */
public final class TurretStateSpaceController {

    /** Model + tuning parameters. Plain fields so tests can build arbitrary models. */
    public static final class Params {
        public double a, b;               // continuous plant: omega' = -a*omega + b*V
        public double samplePeriod = TurretConfig.SAMPLE_PERIOD_S;
        public double maxAngleError = TurretConfig.MAX_ANGLE_ERROR_RAD;
        public double maxVelocityError = TurretConfig.MAX_VELOCITY_ERROR_RAD_S;
        public double maxVolts = TurretConfig.MAX_CONTROL_VOLTS;
        public double maxRefVelocity = TurretConfig.MAX_REFERENCE_VELOCITY_RAD_S;
        public double velocityAlpha = TurretConfig.VELOCITY_FILTER_ALPHA;
        public double staticFrictionVolts = TurretConfig.STATIC_FRICTION_VOLTS;
        public double integralGain = TurretConfig.INTEGRAL_GAIN;
        public double integralLimitVolts = TurretConfig.INTEGRAL_LIMIT_VOLTS;

        /** Derives a and b from the motor datasheet numbers in {@link TurretConfig}. */
        public static Params fromConfig() {
            Params p = new Params();
            double r = TurretConfig.NOMINAL_VOLTAGE / TurretConfig.STALL_CURRENT_A;
            double kt = TurretConfig.SHAFT_STALL_TORQUE_NM / TurretConfig.STALL_CURRENT_A;
            double kv = TurretConfig.NOMINAL_VOLTAGE / TurretConfig.SHAFT_FREE_SPEED_RAD_S;
            double g = TurretConfig.SHAFT_REVS_PER_TURRET_REV;
            double j = TurretConfig.LOAD_INERTIA_KG_M2;
            p.a = (g * g * kt * kv / r + TurretConfig.VISCOUS_FRICTION) / j;
            p.b = g * kt / (r * j);
            return p;
        }
    }

    private final Params p;
    private final double[][] ad, bd, k;
    private final double bdNormSq;

    // runtime state
    private double refTheta, refOmega, omegaFiltered, integral;
    private boolean initialised;
    private boolean saturated, fault;
    private double lastVolts, lastPower;

    public TurretStateSpaceController(Params params) {
        if (!(params.a >= 0) || !(params.b > 0) || !(params.samplePeriod > 0) || !(params.maxVolts > 0)
                || !(params.maxAngleError > 0) || !(params.maxVelocityError > 0) || !(params.maxRefVelocity > 0)
                || !(params.velocityAlpha > 0 && params.velocityAlpha <= 1)
                || !Double.isFinite(params.a) || !Double.isFinite(params.b)) {
            throw new IllegalArgumentException("invalid turret controller parameters");
        }
        this.p = params;
        double[][] ac = {{0, 1}, {0, -params.a}};
        double[][] bc = {{0}, {params.b}};
        double[][][] d = Matrices.discretize(ac, bc, params.samplePeriod);
        ad = d[0];
        bd = d[1];
        double[][] q = {{1 / sq(params.maxAngleError), 0}, {0, 1 / sq(params.maxVelocityError)}};
        double[][] r = {{1 / sq(params.maxVolts)}};
        k = Lqr.dlqr(ad, bd, q, r);
        bdNormSq = bd[0][0] * bd[0][0] + bd[1][0] * bd[1][0];
    }

    private static double sq(double v) { return v * v; }

    /** Feedback gains [Kθ, Kω] in volts per rad and volts per rad/s. */
    public double[] gains() { return new double[]{k[0][0], k[0][1]}; }
    public double[][] discreteA() { return Matrices.copy(ad); }
    public double[][] discreteB() { return Matrices.copy(bd); }

    /** Forget internal state (call when the controller is (re)started, e.g. after a fault). */
    public void reset() {
        initialised = false;
        integral = 0;
        omegaFiltered = 0;
        saturated = false;
        fault = false;
        lastVolts = lastPower = 0;
    }

    /**
     * One control step.
     *
     * @param desiredAngle reachable, unwrapped target angle (rad)
     * @param desiredVelocity rate of change of the target angle caused by robot motion (rad/s)
     * @param measuredAngle  turret angle (rad), unwrapped, same frame as desiredAngle
     * @param measuredVelocity raw turret angular velocity (rad/s)
     * @param batteryVolts   current battery voltage (V); scales volts to the [-1, 1] power FTC expects
     * @param dt             time since the previous call (s), used to slew the reference only
     * @return motor power in [-1, 1]; 0 and {@link #hasFault()} on any invalid input
     */
    public double update(double desiredAngle, double desiredVelocity, double measuredAngle,
                         double measuredVelocity, double batteryVolts, double dt) {
        if (!Double.isFinite(desiredAngle) || !Double.isFinite(desiredVelocity) || !Double.isFinite(measuredAngle)
                || !Double.isFinite(measuredVelocity) || !(batteryVolts > 1.0) || !Double.isFinite(batteryVolts)
                || !(dt > 0) || !Double.isFinite(dt)) {
            fault = true;
            lastVolts = lastPower = 0;
            saturated = false;
            initialised = false;   // re-seed from the measurement once inputs are valid again
            return 0;
        }
        fault = false;
        if (!initialised) {
            refTheta = measuredAngle;
            refOmega = 0;
            omegaFiltered = measuredVelocity;
            integral = 0;
            initialised = true;
        } else {
            omegaFiltered += p.velocityAlpha * (measuredVelocity - omegaFiltered);
        }

        // Slew-limited reference so a target switch is a bounded-velocity move, not a step.
        double step = p.maxRefVelocity * dt;
        double toGo = desiredAngle - refTheta;
        double desVel = AngleUtil.clamp(desiredVelocity, -p.maxRefVelocity, p.maxRefVelocity);
        if (Math.abs(toGo) <= step) {
            refTheta = desiredAngle;
            refOmega = desVel;
        } else {
            refTheta += Math.signum(toGo) * step;
            refOmega = Math.signum(toGo) * p.maxRefVelocity;
        }

        // Plant-inversion feedforward (least squares over the two states).
        double rn0 = refTheta + refOmega * p.samplePeriod, rn1 = refOmega;
        double e0 = rn0 - (ad[0][0] * refTheta + ad[0][1] * refOmega);
        double e1 = rn1 - (ad[1][0] * refTheta + ad[1][1] * refOmega);
        double uff = (bd[0][0] * e0 + bd[1][0] * e1) / bdNormSq;

        double errTheta = refTheta - measuredAngle, errOmega = refOmega - omegaFiltered;
        double ufb = k[0][0] * errTheta + k[0][1] * errOmega;

        double u = uff + ufb + p.integralGain * integral;
        if (p.staticFrictionVolts != 0 && Math.abs(desiredAngle - measuredAngle) > 1e-3) {
            u += p.staticFrictionVolts * Math.signum(u);
        }

        double limit = Math.min(p.maxVolts, batteryVolts);
        double uSat = AngleUtil.clamp(u, -limit, limit);
        saturated = uSat != u;
        // Anti-windup: only integrate while not pushing further into saturation.
        if (p.integralGain != 0 && !(saturated && Math.signum(errTheta) == Math.signum(u))) {
            double lim = p.integralLimitVolts / Math.abs(p.integralGain);
            integral = AngleUtil.clamp(integral + errTheta * dt, -lim, lim);
        }
        lastVolts = uSat;
        lastPower = uSat / batteryVolts;
        return lastPower;
    }

    public boolean isSaturated() { return saturated; }
    public boolean hasFault() { return fault; }
    public double lastVolts() { return lastVolts; }
    public double lastPower() { return lastPower; }
    public double referenceAngle() { return refTheta; }
    public double filteredVelocity() { return omegaFiltered; }
}
