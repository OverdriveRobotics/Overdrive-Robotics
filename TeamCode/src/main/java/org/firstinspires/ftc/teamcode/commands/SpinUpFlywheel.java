package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

/**
 * Requests a flywheel target velocity (ticks/sec) and finishes once the {@link FlywheelRegulator} reports the
 * velocity stable inside tolerance. It writes no motors itself: the regulator is the only writer, so the target
 * stays commanded after this command ends (naturally, by timeout or by interruption). Use {@link StopFlywheel}
 * to stop the flywheel.
 *
 * No resource requirement: it never competes with the regulator or with shooting/driving commands.
 * Repeated identical requests change nothing. A later request for a different target supersedes this one
 * deterministically: this command ends (not a failure) as soon as the stored target is no longer its own.
 */
class SpinUpFlywheel extends BaseCommand {
    private final RobotHardware robot;
    private final double target, tolerance, stableMs, timeoutMs;
    private double startMs;
    private boolean failed, superseded;

    SpinUpFlywheel(RobotHardware robot, double target, double tolerance, double stableMs, double timeoutMs) {
        super(0, ConflictBehavior.OVERRIDE);
        this.robot = robot;
        this.target = target;
        this.tolerance = tolerance;
        this.stableMs = stableMs;
        this.timeoutMs = timeoutMs;
    }

    @Override public void start() {
        failed = superseded = false;
        startMs = nowMs();
        String err = null;
        if (!FlywheelUtil.hardwareAvailable(robot)) err = "flywheel hardware unavailable";
        else if (!Double.isFinite(target) || target <= 0 || target > RobotConfig.MAX_FLYWHEEL_VELOCITY)
            err = "invalid target velocity " + target;
        else if (!(tolerance > 0) || !Double.isFinite(tolerance)) err = "invalid tolerance " + tolerance;
        else if (!(stableMs >= 0) || !Double.isFinite(stableMs)) err = "invalid stable time " + stableMs;
        else if (!(timeoutMs > 0) || !Double.isFinite(timeoutMs)) err = "invalid timeout " + timeoutMs;
        if (err != null) { fail(err); return; }

        FlywheelUtil.requestTarget(target, tolerance, stableMs);
        FlywheelUtil.ensureRegulator(robot);
    }

    @Override public void execute() {
        if (failed || superseded) return;
        if (Math.abs(Storage.flywheelTargetVelocity - target) > 1e-6) { superseded = true; return; }
        if (Storage.flywheelFault != null) { fail("flywheel fault: " + Storage.flywheelFault); return; }
        if (!Storage.isFlywheelReadyFresh() && nowMs() - startMs > timeoutMs) {
            // Timeouts latch a fault (cleared by the next request) so nothing shoots at an unreached velocity.
            String m = "flywheel did not reach " + target + " within " + timeoutMs + " ms (measured "
                    + Storage.flywheelMeasuredVelocity + ")";
            Storage.flywheelFault = m;
            fail(m);
        }
    }

    @Override public boolean done() {
        return failed || superseded
                || (Math.abs(Storage.flywheelTargetVelocity - target) <= 1e-6 && Storage.isFlywheelReadyFresh());
    }

    @Override public void end(EndCondition endCondition) {
        // Intentionally leaves the target (and so the motors) alone. Readiness is owned by the regulator and
        // is recomputed from the live measurement, so nothing to invalidate here.
    }

    private void fail(String msg) {
        failed = true;
        Storage.flywheelReady = false;
        Storage.reportFailure("SpinUpFlywheel", msg);
    }
}
