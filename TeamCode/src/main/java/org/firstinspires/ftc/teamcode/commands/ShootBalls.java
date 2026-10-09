package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;

import org.firstinspires.ftc.teamcode.control.ShootingReadiness;
import org.firstinspires.ftc.teamcode.hardware.RobotClock;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

/**
 * Shoots {@code balls} balls, one at a time, while the robot may be driving:
 *
 *   WAIT_READY -> OPENING -> FEED -> CLOSING -> RECOVER -> WAIT_READY -> ... -> DONE
 *
 *  WAIT_READY  full live readiness (flywheel stable, plus turret/zone/localization when enabled in RobotConfig) is
 *              re-checked before EVERY ball. If readiness is lost (e.g. the robot leaves the shooting zone) nothing
 *              is fed; driving and tracking continue and the next window is awaited until READY_TIMEOUT_MS.
 *  OPENING     stopper opened; feed motor stays off for STOPPER_SETTLE_MS (calibrated assumption: no servo feedback).
 *  FEED        feed motor on until an RPM drop (>= SHOT_DROP_THRESHOLD below the peak, DROP_CONFIRM_LOOPS loops in a
 *              row) counts the shot. POLICY: once a ball is being fed it is fed to completion even if readiness
 *              drops mid-ball (stopping halfway risks a jam); no further ball starts until readiness is valid again.
 *              A flywheel fault/stale measurement or SHOT_TIMEOUT_MS fails the command.
 *  CLOSING     feed stopped and stopper closed immediately on detection, settle STOPPER_CLOSE_SETTLE_MS. The stopper
 *              closes between shots so the next ball is retained. Skipped after the final ball.
 *  RECOVER     waits for the REGULATOR's measured-velocity readiness (not elapsed time), RECOVER_TIMEOUT_MS.
 *
 * Resources: stopper servo + feed motor (so the intake cannot run concurrently). It never requires the flywheel and
 * never commands or stops it, on success, failure or cancellation. Priority 1 beats RunIntake (0); a second
 * ShootBalls cannot displace a running one (CANCEL).
 */
class ShootBalls extends BaseCommand {
    private enum Phase { WAIT_READY, OPENING, FEED, CLOSING, RECOVER, DONE }

    private final RobotHardware robot;
    private final int balls;
    private final double shootVelocity;   // NaN = use whatever target Storage holds
    private final double tolerance, readyTimeoutMs;

    private Phase phase = Phase.DONE;
    private double phaseStartMs, feedStartMs, peakVelocity;
    private int dropLoops, shot;
    private String failure;
    /** True once validation passed and this command has taken over the stopper/feed hardware. */
    private boolean hwEngaged;

    ShootBalls(RobotHardware robot, int balls, double shootVelocity, double tolerance) {
        this(robot, balls, shootVelocity, tolerance, RobotConfig.READY_TIMEOUT_MS);
    }

    ShootBalls(RobotHardware robot, int balls, double shootVelocity, double tolerance, double readyTimeoutMs) {
        // priority 1: overrides a running intake, and cannot itself be displaced by it.
        super(1, ConflictBehavior.CANCEL, Resources.stopper(robot), Resources.feed(robot));
        this.robot = robot;
        this.balls = balls;
        this.shootVelocity = shootVelocity;
        this.tolerance = tolerance;
        this.readyTimeoutMs = readyTimeoutMs;
    }

    @Override public void start() {
        failure = null;
        shot = 0;
        dropLoops = 0;
        phase = Phase.DONE;
        hwEngaged = false;
        Storage.ballsShot = 0;
        String err = null;
        if (robot == null || robot.stopper == null || robot.intakeAndTransferMotor == null
                || !FlywheelUtil.hardwareAvailable(robot)) err = "shooter hardware unavailable";
        else if (balls < 1 || balls > RobotConfig.MAX_BALLS) err = "invalid ball count " + balls;
        else if (!(tolerance > 0) || !Double.isFinite(tolerance)) err = "invalid tolerance " + tolerance;
        else if (!(readyTimeoutMs > 0) || !Double.isFinite(readyTimeoutMs)) err = "invalid ready timeout " + readyTimeoutMs;
        else if (!Double.isNaN(shootVelocity)
                && (!Double.isFinite(shootVelocity) || shootVelocity <= 0 || shootVelocity > RobotConfig.MAX_FLYWHEEL_VELOCITY))
            err = "invalid shoot velocity " + shootVelocity;
        if (err != null) { fail(err); return; }

        hwEngaged = true;
        Storage.shootInProgress = true;
        closeAndStopFeed();
        // Request (idempotent) rather than command: the regulator is the only flywheel writer.
        if (!Double.isNaN(shootVelocity)) {
            FlywheelUtil.requestTarget(shootVelocity, tolerance, RobotConfig.FLYWHEEL_STABLE_MS);
        }
        FlywheelUtil.ensureRegulator(robot);
        enter(Phase.WAIT_READY);
    }

    @Override public void execute() {
        if (phase == Phase.DONE || failure != null) return;
        double now = nowMs();
        double v = Storage.flywheelMeasuredVelocity;
        boolean fresh = RobotClock.nanos() - Storage.flywheelMeasuredNanos <= Storage.MAX_SAMPLE_AGE_NANOS;

        if (Storage.flywheelFault != null) { fail("flywheel fault: " + Storage.flywheelFault); return; }

        switch (phase) {
            case WAIT_READY: {
                ShootingReadiness.Result res = ShootStatus.evaluate(robot, true, null);
                Storage.readyToShoot = res.ready;
                Storage.notReadyReason = res.reason;
                if (res.ready) {
                    peakVelocity = v;
                    robot.stopper.setPosition(RobotConfig.STOPPER_OPEN);
                    enter(Phase.OPENING);
                } else if (now - phaseStartMs > readyTimeoutMs) {
                    fail("not ready within " + readyTimeoutMs + " ms: " + res.reason);
                }
                break;
            }
            case OPENING:
                if (now - phaseStartMs >= RobotConfig.STOPPER_SETTLE_MS) {
                    robot.intakeAndTransferMotor.setPower(RobotConfig.FEED_POWER);
                    Storage.intakeState = Storage.IntakeState.FEEDING;
                    feedStartMs = now;
                    peakVelocity = Math.max(peakVelocity, v);
                    dropLoops = 0;
                    enter(Phase.FEED);
                }
                break;

            case FEED:
                if (!fresh || !Double.isFinite(v)) { fail("flywheel measurement stale/invalid while feeding"); return; }
                peakVelocity = Math.max(peakVelocity, v);
                if (RobotConfig.CHECK_FEED_ENCODER && now - feedStartMs > RobotConfig.FEED_CHECK_AFTER_MS
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
                    if (shot >= balls) { enter(Phase.DONE); return; }   // no recovery wait after the final ball
                    enter(Phase.CLOSING);
                } else if (now - feedStartMs > RobotConfig.SHOT_TIMEOUT_MS) {
                    fail("no RPM drop detected after feeding (ball " + (shot + 1) + ")");
                }
                break;

            case CLOSING:
                if (now - phaseStartMs >= RobotConfig.STOPPER_CLOSE_SETTLE_MS) enter(Phase.RECOVER);
                break;

            case RECOVER:
                // A temporary drop is expected here; only the timeout is a failure.
                if (Storage.isFlywheelReadyFresh()) enter(Phase.WAIT_READY);
                else if (now - phaseStartMs > RobotConfig.RECOVER_TIMEOUT_MS)
                    fail("flywheel did not recover to " + Storage.flywheelTargetVelocity + " (measured " + v + ")");
                break;

            default:
                break;
        }
    }

    @Override public boolean done() { return phase == Phase.DONE || failure != null; }

    @Override public void end(EndCondition endCondition) {
        // Second safety layer: release only what we own. The flywheel is deliberately left running.
        if (hwEngaged) closeAndStopFeed();
        if (hwEngaged) Storage.shootInProgress = false;
        Storage.shootPhase = "IDLE";
        if (Storage.intakeState == Storage.IntakeState.FEEDING) Storage.intakeState = Storage.IntakeState.IDLE;
        if (failure == null && shot < balls && phase != Phase.DONE) {
            Storage.reportFailure("ShootBalls", "interrupted after " + shot + "/" + balls + " balls");
        }
    }

    private void enter(Phase p) {
        phase = p;
        phaseStartMs = nowMs();
        Storage.shootPhase = p.name();
    }

    private void closeAndStopFeed() {
        robot.intakeAndTransferMotor.setPower(0);
        robot.stopper.setPosition(RobotConfig.STOPPER_CLOSED);
    }

    private void fail(String msg) {
        if (failure != null) return;
        failure = msg;
        phase = Phase.DONE;
        if (hwEngaged) closeAndStopFeed();   // a rejected command (bad parameters) never touches the hardware
        if (hwEngaged) Storage.shootInProgress = false;
        Storage.shootPhase = "FAILED";
        if (Storage.intakeState == Storage.IntakeState.FEEDING) Storage.intakeState = Storage.IntakeState.IDLE;
        Storage.reportFailure("ShootBalls", msg + " (shot " + shot + "/" + balls + ")");
    }
}
