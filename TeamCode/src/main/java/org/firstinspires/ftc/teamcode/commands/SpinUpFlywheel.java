package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

/**
 * Spins the flywheel to a target velocity (ticks/sec) and finishes once it has been within tolerance for
 * the stable period. Does NOT stop the flywheel when it ends (or is interrupted): the target stays commanded
 * so other commands can shoot. Use {@link StopFlywheel} to stop it.
 *
 * Resources: the flywheel. A second spin-up OVERRIDEs this one; since the motors are only re-commanded when
 * the target actually changes, a repeated request for the same target does not disturb the running flywheel.
 */
class SpinUpFlywheel extends BaseCommand {
    private final RobotHardware robot;
    private final double target, tolerance, stableMs, timeoutMs;
    private double startMs, inBandSinceMs;
    private boolean failed;

    SpinUpFlywheel(RobotHardware robot, double target, double tolerance, double stableMs, double timeoutMs) {
        super(0, ConflictBehavior.OVERRIDE, FlywheelUtil.flywheelKey(robot));
        this.robot = robot;
        this.target = target;
        this.tolerance = tolerance;
        this.stableMs = stableMs;
        this.timeoutMs = timeoutMs;
    }

    @Override public void start() {
        failed = false;
        startMs = nowMs();
        inBandSinceMs = -1;
        String err = null;
        if (!FlywheelUtil.hardwareAvailable(robot)) err = "flywheel hardware unavailable";
        else if (!Double.isFinite(target) || target <= 0 || target > RobotConfig.MAX_FLYWHEEL_VELOCITY)
            err = "invalid target velocity " + target;
        else if (!(tolerance > 0)) err = "invalid tolerance " + tolerance;
        if (err != null) { fail(err); return; }

        // Read before applying power: the flywheel may already be spinning; keep it that way.
        double current = FlywheelUtil.sample(robot);
        if (!Double.isFinite(current)) { fail("invalid flywheel velocity reading"); return; }

        boolean targetChanged = Math.abs(Storage.flywheelTargetVelocity - target) > 1e-6;
        if (targetChanged || Storage.flywheelFault != null) {
            Storage.flywheelReady = false;
            Storage.flywheelFault = null;
            Storage.flywheelTargetVelocity = target;
            robot.shooterLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.shooterRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.shooterLeft.setVelocity(target);
            robot.shooterRight.setVelocity(target);
        }
        // else: same target already commanded -> leave motors and readiness untouched.
    }

    @Override public void execute() {
        if (failed) return;
        double v = FlywheelUtil.sample(robot);
        if (!Double.isFinite(v)) { fail("invalid flywheel velocity reading"); return; }
        double now = nowMs();
        if (FlywheelUtil.inTolerance(v, target, tolerance)) {
            if (inBandSinceMs < 0) inBandSinceMs = now;
            Storage.flywheelReady = (now - inBandSinceMs) >= stableMs;
        } else {
            inBandSinceMs = -1;
            Storage.flywheelReady = false;
        }
        if (!Storage.flywheelReady && now - startMs > timeoutMs) {
            fail("flywheel did not reach " + target + " within " + timeoutMs + " ms (measured " + v + ")");
        }
    }

    @Override public boolean done() { return failed || Storage.flywheelReady && Storage.flywheelTargetVelocity == target; }

    @Override public void end(EndCondition endCondition) {
        // Intentionally leaves the shooter motors running. Only readiness is invalidated if we did not finish cleanly.
        if (endCondition != EndCondition.NATURALLY) Storage.flywheelReady = false;
    }

    private void fail(String msg) {
        failed = true;
        Storage.flywheelReady = false;
        Storage.flywheelFault = msg;
        Storage.reportFailure("SpinUpFlywheel", msg);
    }
}
