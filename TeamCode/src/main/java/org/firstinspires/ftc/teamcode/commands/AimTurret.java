package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.control.TurretConfig;
import org.firstinspires.ftc.teamcode.control.TurretStateSpaceController;
import org.firstinspires.ftc.teamcode.control.TurretUtil;
import org.firstinspires.ftc.teamcode.hardware.RobotClock;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

/**
 * Continuously aims the turret at the ACTIVE field target (Storage.activeTarget()) using the live Pedro pose.
 * Runs until cancelled. The target is re-read every loop, so switching targets at runtime just changes the
 * reference (the controller slews to it) with no restart and no motor-command conflict.
 *
 * Resource: the turret motor only; runs in parallel with path following, flywheel control and shooting.
 *
 * SAFETY: the motor is only powered when {@link TurretConfig#HARDWARE_CONFIGURED} is true (gear ratio, encoder,
 * limits filled in). Otherwise it publishes telemetry/status, holds the motor at 0 and reports why.
 * On a bad pose, invalid target, stale pose, unreadable battery or controller fault the output is 0.
 */
class AimTurret extends BaseCommand {
    private final RobotHardware robot;
    private TurretStateSpaceController controller;
    private long lastNanos;
    private boolean failedToStart;

    AimTurret(RobotHardware robot) {
        super(0, ConflictBehavior.OVERRIDE, Resources.turret(robot));
        this.robot = robot;
    }

    @Override public void start() {
        failedToStart = false;
        lastNanos = 0;
        if (robot == null || robot.turret == null || robot.follower == null) {
            failedToStart = true;
            Storage.turretFault = "turret hardware unavailable";
            Storage.reportFailure("AimTurret", Storage.turretFault);
            return;
        }
        try {
            controller = new TurretStateSpaceController(TurretStateSpaceController.Params.fromConfig());
        } catch (RuntimeException e) {
            failedToStart = true;
            Storage.turretFault = "turret controller design failed: " + e.getMessage();
            Storage.reportFailure("AimTurret", Storage.turretFault);
            return;
        }
        // Raw power: the SDK's RUN_USING_ENCODER velocity loop would fight the voltage model.
        robot.turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.turret.setPower(0);
    }

    @Override public void execute() {
        if (failedToStart) return;
        long nowN = RobotClock.nanos();
        double dt = lastNanos == 0 ? TurretConfig.SAMPLE_PERIOD_S : (nowN - lastNanos) / 1e9;
        lastNanos = nowN;

        double angle = TurretUtil.ticksToAngle(robot.turret.getCurrentPosition() - Storage.turretZeroTicks);
        double vel = TurretUtil.ticksPerSecToRadPerSec(robot.turret.getVelocity());
        Storage.turretAngle = angle;
        Storage.turretVelocity = vel;
        Storage.turretUpdateNanos = nowN;

        String fault = null;
        Pose pose = robot.follower.pose();
        Velocity v = robot.follower.velocity();
        Storage.FieldPoint t = Storage.activeTarget();
        double poseAgeMs = Storage.poseUpdateNanos == 0 ? Double.POSITIVE_INFINITY : (nowN - Storage.poseUpdateNanos) / 1e6;
        if (pose == null || !Double.isFinite(pose.x()) || !Double.isFinite(pose.y()) || !Double.isFinite(pose.heading()))
            fault = "invalid pose";
        else if (!(poseAgeMs <= TurretConfig.MAX_POSE_AGE_MS)) fault = "stale pose";
        else if (t == null || !t.isValid()) fault = "selected target " + Storage.activeTargetIndex + " invalid";
        else if (!Double.isFinite(angle) || !Double.isFinite(vel)) fault = "turret encoder invalid";
        else if (!TurretConfig.CONTINUOUS && (angle < TurretConfig.MIN_ANGLE_RAD - 0.35 || angle > TurretConfig.MAX_ANGLE_RAD + 0.35))
            fault = "turret outside mechanical range";

        double power = 0;
        boolean saturated = false;
        if (fault == null) {
            double desiredRel = TurretUtil.desiredRelativeAngle(pose.x(), pose.y(), pose.heading(), t.x, t.y);
            double fvx = v == null ? 0 : v.vx, fvy = v == null ? 0 : v.vy, omega = v == null ? 0 : v.omega;
            if (!TurretConfig.ROBOT_VELOCITY_IS_FIELD_FRAME) {
                double[] f = TurretUtil.robotToFieldVelocity(fvx, fvy, pose.heading());
                fvx = f[0]; fvy = f[1];
            }
            double rate = TurretUtil.desiredRelativeRate(pose.x(), pose.y(), fvx, fvy, omega, t.x, t.y);
            if (!Double.isFinite(rate)) rate = 0;

            double reachable = TurretUtil.reachableAngle(desiredRel, angle);
            Storage.turretDesiredAngle = desiredRel;
            if (Double.isNaN(reachable)) {
                Storage.turretReachable = false;
                Storage.turretError = Double.NaN;
                controller.reset();
            } else {
                Storage.turretReachable = true;
                power = controller.update(reachable, rate, angle, vel, robot.batteryVolts(), dt);
                if (controller.hasFault()) {
                    fault = "controller input invalid (battery/encoder)";
                    power = 0;
                } else {
                    if (!TurretConfig.CONTINUOUS) {
                        power = TurretUtil.guardLimits(power, angle, TurretConfig.MIN_ANGLE_RAD,
                                TurretConfig.MAX_ANGLE_RAD, TurretConfig.LIMIT_MARGIN_RAD);
                    }
                    saturated = controller.isSaturated();
                    Storage.turretError = reachable - angle;
                }
            }
        } else {
            controller.reset();
            Storage.turretError = Double.NaN;
            Storage.turretReachable = false;
        }

        if (fault == null && !TurretConfig.HARDWARE_CONFIGURED) {
            fault = "turret not calibrated (TurretConfig.HARDWARE_CONFIGURED=false): motor held off";
            power = 0;
        }
        Storage.turretFault = fault;
        Storage.turretSaturated = saturated;
        Storage.turretPower = power;
        robot.turret.setPower(power);
    }

    @Override public boolean done() { return false; }

    @Override public void end(EndCondition endCondition) {
        if (robot != null && robot.turret != null) robot.turret.setPower(0);
        Storage.turretPower = 0;
        Storage.turretError = Double.NaN;
        Storage.turretReachable = false;
    }
}
