package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.info;
import static org.firstinspires.ftc.teamcode.T.near;
import static org.firstinspires.ftc.teamcode.commands.SimSupport.*;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.control.FlywheelConfig;
import org.firstinspires.ftc.teamcode.control.TurretConfig;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

import java.util.HashSet;
import java.util.Set;

/** Simulation-only tests (real Ivy scheduler, mocked hardware, fake clock). */
public final class MovingShotTests {
    private MovingShotTests() {}

    public static void run() {
        sim("moving shot: path following, flywheel, turret aiming and shooting run concurrently; shooting does not wait for the path", s -> {
            TurretConfig.HARDWARE_CONFIGURED = true;
            RobotConfig.REQUIRE_TURRET_ALIGNED = true;
            RobotConfig.REQUIRE_LOCALIZATION = true;
            Storage.setTarget(0, 60, 40);
            s.follower.pose = new Pose(10, 10, 0);
            s.follower.vel = new Velocity(5, 0, 0);
            s.ballsLoaded = 3;
            RobotCommands.startShooterSystems(s.robot);
            Command move = RobotCommands.movingShot(s.robot, null, 3, 1800, 6000);
            Scheduler.schedule(move);
            int shotsWhenPathEnded = -1;
            for (int i = 0; i < 1500 && running(move); i++) {
                s.follower.pose = new Pose(10 + 0.05 * i, 10, 0);   // still driving
                s.step(10);
                if (i == 600) { s.follower.atEnd = true; shotsWhenPathEnded = Storage.ballsShot; }
            }
            info("shots completed before the path ended: " + shotsWhenPathEnded + ", total " + Storage.ballsShot);
            check(shotsWhenPathEnded >= 1, "shooting began while the path was still being followed");
            check(Storage.ballsShot == 3, "all balls shot: " + Storage.lastFailure);
            check(s.follower.followCalls == 1, "path started once");
            check(!running(move), "group finished");
            check(Storage.turretFault == null || !Storage.turretFault.contains("conflict"), "no conflict");
            // background loops were not displaced by the shot / path:
            s.run(100);
            check(Storage.flywheelReady, "regulator still holding the flywheel");
            check(Math.abs(Storage.turretError) < Math.toRadians(3), "turret still tracking: " + Math.toDegrees(Storage.turretError));
            assertSafe(s, "end");
        });

        sim("moving shot: times out safely if the valid window never occurs, path keeps going", s -> {
            RobotConfig.SHOOT_ZONE_ENABLED = true;
            RobotConfig.REQUIRE_LOCALIZATION = true;
            RobotConfig.SHOOT_ZONE_MIN_X = 100; RobotConfig.SHOOT_ZONE_MAX_X = 120;
            RobotConfig.SHOOT_ZONE_MIN_Y = 100; RobotConfig.SHOOT_ZONE_MAX_Y = 120;
            s.follower.pose = new Pose(10, 10, 0);
            s.ballsLoaded = 1;
            RobotCommands.startShooterSystems(s.robot);
            Command move = RobotCommands.movingShot(s.robot, null, 1, 1800, 800);
            Scheduler.schedule(move);
            s.run(1500);
            check(running(move), "group still alive until the path ends");
            check(Storage.lastFailure != null && Storage.lastFailure.contains("outside shooting zone"), "shot gave up: " + Storage.lastFailure);
            assertSafe(s, "after window timeout");
            s.follower.atEnd = true;
            s.run(50);
            check(!running(move), "group ends once the path ends");
        });

    }
}
