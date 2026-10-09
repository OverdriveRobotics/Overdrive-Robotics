package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.FlywheelConfig;
import org.firstinspires.ftc.teamcode.control.FlywheelLqrController;
import org.firstinspires.ftc.teamcode.control.TurretConfig;
import org.firstinspires.ftc.teamcode.control.TurretStateSpaceController;
import org.firstinspires.ftc.teamcode.control.TurretUtil;
import org.firstinspires.ftc.teamcode.hardware.RobotClock;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Proxy;
import java.util.function.BooleanSupplier;

/**
 * Simulated robot: proxy-mocked FTC hardware + a crude physical model, driven in lock-step with the REAL Ivy
 * scheduler and a fake clock. Simulation only - none of this validates real hardware.
 */
final class SimRobot {
    static final class Motor implements InvocationHandler {
        double power, velCmd, velocityReading, currentAmps;
        final java.util.List<String> writers = new java.util.ArrayList<>();   // class names that wrote power/velocity
        int velCalls, powerWrites, position;
        Object mode = DcMotor.RunMode.RUN_USING_ENCODER;
        void noteWriter() { noteWriterInto(writers); }

        @Override public Object invoke(Object proxy, java.lang.reflect.Method m, Object[] a) {
            switch (m.getName()) {
                case "setPower": power = (Double) a[0]; powerWrites++; noteWriter(); if (mode == DcMotor.RunMode.RUN_USING_ENCODER) velCmd = power * 2800; return null;
                case "getPower": return power;
                case "setVelocity": if (a.length == 1) { velCmd = (Double) a[0]; velCalls++; noteWriter(); } return null;
                case "getVelocity": return a == null || a.length == 0 ? velocityReading : 0.0;
                case "setMode": mode = a[0]; return null;
                case "getMode": return mode;
                case "getCurrentPosition": return position;
                case "getCurrent": return power != 0 ? currentAmps : 0.0;
                case "hashCode": return System.identityHashCode(proxy);
                case "equals": return proxy == a[0];
                case "toString": return "Motor";
                default: return defaultFor(m.getReturnType());
            }
        }
    }

    /** Records which production class called a motor write (first non-test, non-proxy frame). */
    static void noteWriterInto(java.util.List<String> into) {
        for (StackTraceElement e : Thread.currentThread().getStackTrace()) {
            String cn = e.getClassName();
            if (cn.startsWith("org.firstinspires.ftc.teamcode.") && !cn.contains("SimRobot") && !cn.contains("CommandTests")) {
                String simple = cn.substring(cn.lastIndexOf('.') + 1);
                int d = simple.indexOf('$'); if (d > 0) simple = simple.substring(0, d);
                if (!into.contains(simple)) into.add(simple);
                return;
            }
        }
    }

    static final class ServoMock implements InvocationHandler {
        double position = Double.NaN;
        int writes;
        @Override public Object invoke(Object proxy, java.lang.reflect.Method m, Object[] a) {
            switch (m.getName()) {
                case "setPosition": position = (Double) a[0]; writes++; return null;
                case "getPosition": return position;
                case "hashCode": return System.identityHashCode(proxy);
                case "equals": return proxy == a[0];
                case "toString": return "Servo";
                default: return defaultFor(m.getReturnType());
            }
        }
    }

    static Object defaultFor(Class<?> t) {
        if (t == boolean.class) return false;
        if (t == int.class) return 0;
        if (t == long.class) return 0L;
        if (t == double.class) return 0.0;
        if (t == float.class) return 0f;
        if (t == short.class) return (short) 0;
        if (t == byte.class) return (byte) 0;
        return null;
    }

    /** Follower with no constructor run (Unsafe) and only the methods the code under test calls. */
    static class StubFollower extends Follower {
        Pose pose = new Pose(0, 0, 0);
        Velocity vel = new Velocity(0, 0, 0);
        int updates, followCalls, holdCalls, loopsSinceFollow, followDurationLoops;   // followDurationLoops > 0: path ends by itself
        boolean atEnd;
        StubFollower() { super(null, null, null); }
        @Override public Pose pose() { return pose; }
        @Override public Velocity velocity() { return vel; }
        @Override public void update() { updates++; loopsSinceFollow++; }
        @Override public void follow(Path p) { followCalls++; loopsSinceFollow = 0; if (followDurationLoops > 0) atEnd = false; }
        @Override public void hold(Pose p) { holdCalls++; }
        @Override public void setPose(Pose p) { pose = p; }
        @Override public boolean atParametricEnd() { return followDurationLoops > 0 ? loopsSinceFollow >= followDurationLoops : atEnd; }
    }

    static StubFollower newFollower() {
        try {
            // Reflective so this also compiles against Android's bootclasspath (no sun.misc there).
            Class<?> u = Class.forName("sun.misc.Unsafe");
            java.lang.reflect.Field f = u.getDeclaredField("theUnsafe");
            f.setAccessible(true);
            StubFollower sf = (StubFollower) u.getMethod("allocateInstance", Class.class).invoke(f.get(null), StubFollower.class);
            sf.pose = new Pose(0, 0, 0);       // allocateInstance skips field initialisers
            sf.vel = new Velocity(0, 0, 0);
            return sf;
        } catch (Exception e) { throw new RuntimeException(e); }
    }

    // ---- state ----
    long nanos = 1_000_000_000L;
    double vbat = 12.0;
    final RobotHardware robot = new RobotHardware();
    final Motor shooterL = new Motor(), shooterR = new Motor(), intake = new Motor(), turretM = new Motor();
    final ServoMock stopper = new ServoMock(), hood = new ServoMock();
    final StubFollower follower = newFollower();

    // flywheel plant
    double omega;
    boolean flywheelDead, nanVelocity, nanRightOnly, stampPose = true;
    double rightOffset;                    // right shooter reads this much more than the left (ticks/s)
    double sdkTau = 0.25, dropPerBall = 250, ballDelay = 0.15;
    int ballsLoaded, shotsFired;
    double plantScale = 1.0;
    double noiseStd;                       // gaussian velocity-measurement noise (ticks/s), seeded
    final java.util.Random rng = new java.util.Random(1234);
    private double feedTime;
    private FlywheelLqrController.Params fp = FlywheelLqrController.Params.physical();

    // turret plant
    double theta, thetaDot;
    private TurretStateSpaceController.Params tp = TurretStateSpaceController.Params.fromConfig();

    /** Re-derives the simulated plants from the CURRENT config (call after changing plant-defining parameters). */
    void refreshPlants() { fp = FlywheelLqrController.Params.physical(); tp = TurretStateSpaceController.Params.fromConfig(); }
    double flywheelA() { return fp.a; }
    double flywheelB() { return fp.b; }

    SimRobot() {
        robot.shooterLeft = proxy(DcMotorEx.class, shooterL);
        robot.shooterRight = proxy(DcMotorEx.class, shooterR);
        robot.intakeAndTransferMotor = proxy(DcMotorEx.class, intake);
        robot.turret = proxy(DcMotorEx.class, turretM);
        robot.stopper = proxy(Servo.class, stopper);
        robot.hood = proxy(Servo.class, hood);
        robot.follower = follower;
        robot.batteryVoltage = (VoltageSensor) Proxy.newProxyInstance(getClass().getClassLoader(),
                new Class<?>[]{VoltageSensor.class}, (p, m, a) -> m.getName().equals("getVoltage") ? vbat
                        : m.getName().equals("hashCode") ? 7 : m.getName().equals("equals") ? p == a[0] : defaultFor(m.getReturnType()));
    }

    @SuppressWarnings("unchecked")
    private static <T> T proxy(Class<T> c, InvocationHandler h) {
        return (T) Proxy.newProxyInstance(SimRobot.class.getClassLoader(), new Class<?>[]{c}, h);
    }

    /** Resets all shared/static state and installs the fake clock. Returns a fresh simulated robot. */
    static SimRobot fresh() {
        Scheduler.reset();
        org.firstinspires.ftc.teamcode.tuning.ParamRegistry.resetToDefaults();
        Storage.invalidateHardwareState();
        Storage.totalBallsShot = 0;
        Storage.targetA = new Storage.FieldPoint(Double.NaN, Double.NaN);
        Storage.targetB = new Storage.FieldPoint(Double.NaN, Double.NaN);
        Storage.activeTargetIndex = 0;
        Storage.turretZeroTicks = 0;
        RobotConfig.REQUIRE_LOCALIZATION = false;
        RobotConfig.REQUIRE_TURRET_ALIGNED = false;
        RobotConfig.SHOOT_ZONE_ENABLED = false;
        TurretConfig.HARDWARE_CONFIGURED = false;
        FlywheelConfig.MODEL_IDENTIFIED = false;
        SimRobot s = new SimRobot();
        RobotClock.setSource(() -> s.nanos);
        return s;
    }

    void cleanup() { RobotClock.setSource(null); Scheduler.reset(); }

    /** One 'loop': localization update, scheduler, then physics advance. */
    void step(int ms) {
        if (stampPose) robot.updateLocalization();
        publishSensors();
        Scheduler.execute();
        physics(ms / 1000.0);
        nanos += ms * 1_000_000L;
        publishSensors();   // a read right after loop() sees the fresh state, like the real robot after its loop delay
    }

    void run(int totalMs) { for (int t = 0; t < totalMs; t += 10) step(10); }

    /** Steps until cond is true or maxMs elapse. Returns true if cond became true. */
    boolean runUntil(BooleanSupplier cond, int maxMs) {
        for (int t = 0; t < maxMs; t += 10) {
            if (cond.getAsBoolean()) return true;
            step(10);
        }
        return cond.getAsBoolean();
    }

    private void publishSensors() {
        double reading = nanVelocity ? Double.NaN : omega + (noiseStd > 0 ? rng.nextGaussian() * noiseStd : 0);
        shooterL.velocityReading = reading;
        shooterR.velocityReading = nanRightOnly ? Double.NaN : reading + rightOffset;
        turretM.position = (int) Math.round(TurretUtil.angleToTicks(theta, TurretConfig.SHAFT_TICKS_PER_REV,
                TurretConfig.SHAFT_REVS_PER_TURRET_REV, TurretConfig.ENCODER_SIGN, TurretConfig.START_ANGLE_RAD));
        turretM.velocityReading = TurretConfig.ENCODER_SIGN * thetaDot * TurretConfig.SHAFT_REVS_PER_TURRET_REV
                * TurretConfig.SHAFT_TICKS_PER_REV / (2 * Math.PI);
    }

    private void physics(double dt) {
        // ---- flywheel ----
        if (flywheelDead) {
            omega *= Math.exp(-dt / 0.5);
        } else if (shooterL.mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            double v = Math.max(0, Math.min(1, shooterL.power)) * vbat;
            double e = Math.exp(-fp.a * plantScale * dt);
            omega = e * omega + fp.b * (1 - e) / (fp.a * plantScale) * v;
        } else {
            omega += (shooterL.velCmd - omega) * (1 - Math.exp(-dt / sdkTau));
        }
        // ---- ball launch: stopper open + feed running + a ball present ----
        boolean feeding = Math.abs(stopper.position - RobotConfig.STOPPER_OPEN) < 1e-9 && intake.power > 0;
        if (feeding && ballsLoaded > 0) {
            feedTime += dt;
            if (feedTime >= ballDelay) { omega -= dropPerBall; ballsLoaded--; shotsFired++; feedTime = 0; }
        } else if (!feeding) {
            feedTime = 0;
        }
        // ---- turret (exact ZOH of the model the controller assumes) ----
        double v = Math.max(-1, Math.min(1, turretM.power)) * vbat;
        double e = Math.exp(-tp.a * dt);
        double newDot = e * thetaDot + tp.b * (1 - e) / tp.a * v;
        theta += thetaDot * (1 - e) / tp.a + tp.b * v * (dt - (1 - e) / tp.a) / tp.a;
        thetaDot = newDot;
    }
}
