package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

/**
 * Shoots {@code balls} balls, one at a time:
 * WAIT_READY -> (open stopper, feed) FEED -> shot detected (velocity drop) -> (close stopper, stop feed) RECOVER -> next.
 *
 * Resources: stopper servo + intake/feed motor (so intake cannot run at the same time). It does NOT require the
 * flywheel, so driving and spin-up commands run alongside it; it only reads the flywheel and never commands it,
 * except to schedule a spin-up when {@code shootVelocity} is given and differs from the current target.
 * The flywheel is never stopped here, on success or failure.
 */
class ShootBalls extends BaseCommand {
    private enum Phase { WAIT_READY, FEED, RECOVER, DONE }

    private final RobotHardware robot;
    private final int balls;
    private final double shootVelocity;   // NaN = use whatever target Storage holds
    private final double tolerance;

    private Phase phase;
    private double phaseStartMs, inBandSinceMs, peakVelocity, openedAtMs;
    private int dropLoops, shot;
    private String failure;

    ShootBalls(RobotHardware robot, int balls, double shootVelocity, double tolerance) {
        // priority 1: overrides a running intake, and cannot itself be displaced by it.
        super(1, ConflictBehavior.CANCEL, robot.stopper, robot.intakeAndTransferMotor);
        this.robot = robot;
        this.balls = balls;
        this.shootVelocity = shootVelocity;
        this.tolerance = tolerance;
    }

    @Override public void start() {
        failure = null;
        shot = 0;
        dropLoops = 0;
        Storage.ballsShot = 0;
        String err = null;
        if (robot == null || robot.stopper == null || robot.intakeAndTransferMotor == null
                || !FlywheelUtil.hardwareAvailable(robot)) err = "shooter hardware unavailable";
        else if (balls < 1 || balls > RobotConfig.MAX_BALLS) err = "invalid ball count " + balls;
        else if (!(tolerance > 0)) err = "invalid tolerance " + tolerance;
        else if (!Double.isNaN(shootVelocity)
                && (shootVelocity <= 0 || shootVelocity > RobotConfig.MAX_FLYWHEEL_VELOCITY))
            err = "invalid shoot velocity " + shootVelocity;
        if (err != null) { fail(err); return; }

        Storage.shootInProgress = true;
        closeAndStopFeed();
        // Request spin-up through Ivy only if the flywheel isn't already commanded to that velocity (idempotent).
        if (!Double.isNaN(shootVelocity) && Math.abs(Storage.flywheelTargetVelocity - shootVelocity) > 1e-6) {
            Scheduler.schedule(RobotCommands.spinUpFlywheel(robot, shootVelocity));
        }
        enter(Phase.WAIT_READY);
    }

    @Override public void execute() {
        if (phase == Phase.DONE || failure != null) return;
        double now = nowMs();
        double target = Storage.flywheelTargetVelocity;
        double v = FlywheelUtil.sample(robot);   // always verify against live hardware, never just the stored flag

        if (!Double.isFinite(v)) { fail("invalid flywheel velocity reading"); return; }
        if (Storage.flywheelFault != null) { fail("flywheel fault: " + Storage.flywheelFault); return; }
        boolean inBand = FlywheelUtil.inTolerance(v, target, tolerance);
        boolean stable = updateStable(inBand, now);
        // Readiness must follow measured velocity (drops after a shot, returns on recovery).
        Storage.flywheelReady = stable;

        switch (phase) {
            case WAIT_READY:
                if (target <= 0) { fail("flywheel has no target velocity"); return; }
                if (stable) {
                    peakVelocity = v;
                    dropLoops = 0;
                    openedAtMs = now;
                    robot.stopper.setPosition(RobotConfig.STOPPER_OPEN);
                    robot.intakeAndTransferMotor.setPower(RobotConfig.FEED_POWER);
                    Storage.intakeState = Storage.IntakeState.FEEDING;
                    enter(Phase.FEED);
                } else if (now - phaseStartMs > RobotConfig.READY_TIMEOUT_MS) {
                    fail("flywheel never reached shooting velocity " + target + " (measured " + v + ")");
                }
                break;

            case FEED:
                if (!stopperSettled(now)) break;   // let the servo move before judging a drop
                peakVelocity = Math.max(peakVelocity, v);
                if (RobotConfig.CHECK_FEED_ENCODER && now - openedAtMs > RobotConfig.FEED_CHECK_AFTER_MS
                        && Math.abs(robot.intakeAndTransferMotor.getVelocity()) < RobotConfig.FEED_MIN_VELOCITY) {
                    fail("feed mechanism not moving");
                    return;
                }
                dropLoops = (peakVelocity - v >= RobotConfig.SHOT_DROP_THRESHOLD) ? dropLoops + 1 : 0;
                if (dropLoops >= RobotConfig.DROP_CONFIRM_LOOPS) {
                    shot++;
                    Storage.ballsShot = shot;
                    Storage.totalBallsShot++;
                    closeAndStopFeed();
                    Storage.flywheelReady = false;
                    inBandSinceMs = -1;
                    if (shot >= balls) { phase = Phase.DONE; return; }
                    enter(Phase.RECOVER);
                } else if (now - openedAtMs > RobotConfig.SHOT_TIMEOUT_MS) {
                    fail("no RPM drop detected after feeding (ball " + (shot + 1) + ")");
                }
                break;

            case RECOVER:
                // A temporary drop is expected here; only a timeout is a failure.
                if (stable) enter(Phase.WAIT_READY);
                else if (now - phaseStartMs > RobotConfig.RECOVER_TIMEOUT_MS)
                    fail("flywheel did not recover to " + target + " (measured " + v + ")");
                break;

            default:
                break;
        }
    }

    @Override public boolean done() { return phase == Phase.DONE || failure != null; }

    @Override public void end(EndCondition endCondition) {
        // Release only what we own. The flywheel is deliberately left running.
        if (robot != null && robot.stopper != null && robot.intakeAndTransferMotor != null) closeAndStopFeed();
        Storage.shootInProgress = false;
        if (Storage.intakeState == Storage.IntakeState.FEEDING) Storage.intakeState = Storage.IntakeState.IDLE;
        if (failure == null && shot < balls) {
            Storage.reportFailure("ShootBalls", "interrupted after " + shot + "/" + balls + " balls");
        }
    }

    private boolean stopperSettled(double now) { return now - openedAtMs >= RobotConfig.STOPPER_SETTLE_MS; }

    private boolean updateStable(boolean inBand, double now) {
        if (!inBand) { inBandSinceMs = -1; return false; }
        if (inBandSinceMs < 0) inBandSinceMs = now;
        return now - inBandSinceMs >= RobotConfig.FLYWHEEL_STABLE_MS;
    }

    private void enter(Phase p) { phase = p; phaseStartMs = nowMs(); }

    private void closeAndStopFeed() {
        robot.intakeAndTransferMotor.setPower(0);
        robot.stopper.setPosition(RobotConfig.STOPPER_CLOSED);
    }

    private void fail(String msg) {
        failure = msg;
        phase = Phase.DONE;
        Storage.reportFailure("ShootBalls", msg + " (shot " + shot + "/" + balls + ")");
    }
}
