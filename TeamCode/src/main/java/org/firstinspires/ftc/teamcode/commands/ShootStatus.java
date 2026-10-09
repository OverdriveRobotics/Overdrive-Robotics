package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.control.ShootingReadiness;
import org.firstinspires.ftc.teamcode.control.TurretConfig;
import org.firstinspires.ftc.teamcode.control.TurretUtil;
import org.firstinspires.ftc.teamcode.hardware.RobotClock;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

/** Gathers live measurements from Storage/follower and runs the pure {@link ShootingReadiness} evaluation. */
public final class ShootStatus {
    private ShootStatus() {}

    /** Live evaluation. Nothing is cached: call it every loop you care about. */
    public static ShootingReadiness.Result evaluate(RobotHardware robot, boolean feedPermits, String feedReason) {
        ShootingReadiness.Inputs in = new ShootingReadiness.Inputs();
        long now = RobotClock.nanos();

        in.requireLocalization = RobotConfig.REQUIRE_LOCALIZATION;
        in.requireZone = RobotConfig.SHOOT_ZONE_ENABLED;
        in.requireAim = RobotConfig.REQUIRE_TURRET_ALIGNED;
        if (in.requireLocalization || in.requireZone || in.requireAim) {
            if (robot != null && robot.follower != null) {
                com.pedropathing.math.Pose pose = robot.follower.pose();
                com.pedropathing.math.Velocity vel = robot.follower.velocity();
                in.poseFinite = pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                        && Double.isFinite(pose.heading());
                if (in.poseFinite) { in.x = pose.x(); in.y = pose.y(); }
                in.speed = vel == null ? Double.NaN : Math.hypot(vel.vx, vel.vy);
            }
            in.poseAgeMs = Storage.poseUpdateNanos == 0 ? Double.POSITIVE_INFINITY
                    : (now - Storage.poseUpdateNanos) / 1e6;
            in.maxSpeed = RobotConfig.MAX_SHOOT_SPEED;
        }
        in.minX = RobotConfig.SHOOT_ZONE_MIN_X; in.maxX = RobotConfig.SHOOT_ZONE_MAX_X;
        in.minY = RobotConfig.SHOOT_ZONE_MIN_Y; in.maxY = RobotConfig.SHOOT_ZONE_MAX_Y;

        Storage.FieldPoint t = Storage.activeTarget();
        in.targetValid = t != null && t.isValid();
        double turretAgeMs = (now - Storage.turretUpdateNanos) / 1e6;
        in.turretSensorValid = Storage.turretUpdateNanos != 0 && turretAgeMs <= TurretConfig.MAX_POSE_AGE_MS
                && Double.isFinite(Storage.turretAngle);
        in.turretReachable = Storage.turretReachable;
        in.turretErrorRad = Storage.turretError;
        in.turretVelRadS = Storage.turretVelocity;
        in.turretFault = Storage.turretFault;

        in.flywheelTarget = Storage.flywheelTargetVelocity;
        in.flywheelStableFresh = Storage.isFlywheelReadyFresh();
        in.flywheelFault = Storage.flywheelFault;
        in.feedPermits = feedPermits;
        in.feedBlockReason = feedReason;
        return ShootingReadiness.evaluate(in);
    }
}
