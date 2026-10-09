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
public final class ShootingStateMachineTests {
    private ShootingStateMachineTests() {}

    public static void run() {
        sim("shoot: single ball succeeds, stopper closed, feed off, flywheel left running", s -> {
            s.ballsLoaded = 1;
            Command c = RobotCommands.shoot(s.robot, 1, 1800);
            Scheduler.schedule(c);
            check(s.runUntil(() -> !running(c), 8000), "finishes");
            check(Storage.ballsShot == 1 && s.shotsFired == 1, "one shot counted: " + Storage.ballsShot);
            assertSafe(s, "after shot");
            near(1800, Storage.flywheelTargetVelocity, 0, "flywheel target untouched");
            check(Storage.lastFailure == null, "no failure: " + Storage.lastFailure);
        });

        sim("shoot: three balls with recovery between them, no recovery wait after the last", s -> {
            s.ballsLoaded = 3;
            Command c = RobotCommands.shoot(s.robot, 3, 1800);
            Scheduler.schedule(c);
            Set<String> phases = new HashSet<>();
            int recoverEntries = 0, closedWhileRecover = 0;
            String prev = "";
            double lastTarget = 1800;
            for (int i = 0; i < 2000 && running(c); i++) {
                s.step(10);
                String ph = Storage.shootPhase;
                phases.add(ph);
                if (ph.equals("RECOVER") && !prev.equals("RECOVER")) recoverEntries++;
                if (ph.equals("RECOVER") && s.stopper.position == RobotConfig.STOPPER_CLOSED && s.intake.power == 0) closedWhileRecover++;
                if (Storage.flywheelTargetVelocity != lastTarget) throw new AssertionError("flywheel target changed");
                prev = ph;
            }
            info("phases seen: " + phases + ", recoveries: " + recoverEntries);
            check(Storage.ballsShot == 3 && s.shotsFired == 3, "three shots: " + Storage.ballsShot);
            check(phases.containsAll(java.util.Arrays.asList("WAIT_READY", "OPENING", "FEED", "CLOSING", "RECOVER")), "all phases visited");
            check(recoverEntries == 2, "exactly 2 recoveries (none after the last ball): " + recoverEntries);
            check(closedWhileRecover > 0, "stopper closed and feed off while recovering");
            assertSafe(s, "end");
            check(Storage.totalBallsShot == 3, "total count");
        });

        sim("shoot: stopper opens before the feed motor starts (opening settle)", s -> {
            s.ballsLoaded = 1;
            Command c = RobotCommands.shoot(s.robot, 1, 1800);
            Scheduler.schedule(c);
            double openedAt = -1, feedAt = -1;
            for (int i = 0; i < 1500 && running(c); i++) {
                s.step(10);
                if (openedAt < 0 && s.stopper.position == RobotConfig.STOPPER_OPEN) openedAt = s.nanos / 1e6;
                if (feedAt < 0 && s.intake.power > 0) feedAt = s.nanos / 1e6;
            }
            info("open->feed delay " + (feedAt - openedAt) + " ms");
            check(openedAt > 0 && feedAt - openedAt >= RobotConfig.STOPPER_SETTLE_MS - 10, "feed delayed by the opening settle");
        });

        sim("shoot: readiness timeout fails cleanly (flywheel never reaches speed)", s -> {
            s.flywheelDead = true;
            s.ballsLoaded = 1;
            Command c = RobotCommands.shoot(s.robot, 1, 1800, 600);
            Scheduler.schedule(c);
            check(s.runUntil(() -> !running(c), 5000), "ends");
            check(Storage.lastFailure != null && Storage.lastFailure.contains("not ready"), "failure: " + Storage.lastFailure);
            check(s.shotsFired == 0, "never fed");
            assertSafe(s, "after timeout");
        });

        sim("shoot: no RPM drop (no ball) times out and stops the feed", s -> {
            s.ballsLoaded = 0;
            Command c = RobotCommands.shoot(s.robot, 1, 1800);
            Scheduler.schedule(c);
            check(s.runUntil(() -> !running(c), 10000), "ends");
            check(Storage.lastFailure != null && Storage.lastFailure.contains("no RPM drop"), "failure: " + Storage.lastFailure);
            check(Storage.ballsShot == 0, "no shot counted");
            assertSafe(s, "after shot timeout");
        });

        sim("shoot: flywheel recovery timeout fails", s -> {
            s.ballsLoaded = 2;
            Command c = RobotCommands.shoot(s.robot, 2, 1800);
            Scheduler.schedule(c);
            check(s.runUntil(() -> Storage.ballsShot >= 1, 8000), "first ball");
            s.flywheelDead = true;   // flywheel collapses and never recovers
            check(s.runUntil(() -> !running(c), 8000), "ends");
            check(Storage.lastFailure != null && (Storage.lastFailure.contains("recover") || Storage.lastFailure.contains("fault")),
                    "failure: " + Storage.lastFailure);
            check(Storage.ballsShot == 1, "only the first ball counted");
            assertSafe(s, "after recovery failure");
        });

        sim("shoot: cancel during stopper OPENING releases everything, flywheel keeps running", s -> {
            s.ballsLoaded = 1;
            Command c = RobotCommands.shoot(s.robot, 1, 1800);
            Scheduler.schedule(c);
            check(s.runUntil(() -> "OPENING".equals(Storage.shootPhase), 8000), "reached OPENING");
            check(s.stopper.position == RobotConfig.STOPPER_OPEN, "stopper is open");
            Scheduler.cancel(c);
            assertSafe(s, "after cancel in OPENING");
            near(1800, Storage.flywheelTargetVelocity, 0, "flywheel still targeted");
            s.run(300);
            check(Math.abs(s.omega - 1800) < 60, "flywheel still running");
        });

        sim("shoot: cancel during FEED stops the feed and closes the stopper", s -> {
            s.ballsLoaded = 1;
            s.ballDelay = 5;   // ball never leaves on its own
            Command c = RobotCommands.shoot(s.robot, 1, 1800);
            Scheduler.schedule(c);
            check(s.runUntil(() -> "FEED".equals(Storage.shootPhase), 8000), "reached FEED");
            check(s.intake.power > 0, "feeding");
            Scheduler.cancel(c);
            assertSafe(s, "after cancel in FEED");
            near(1800, Storage.flywheelTargetVelocity, 0, "flywheel target kept");
        });

        sim("shoot: an intake started during shooting cannot take the feed motor; shooting completes", s -> {
            s.ballsLoaded = 1;
            Command shoot = RobotCommands.shoot(s.robot, 1, 1800);
            Scheduler.schedule(shoot);
            check(s.runUntil(() -> "FEED".equals(Storage.shootPhase), 8000), "reached FEED");
            Command intake = RobotCommands.runIntake(s.robot, false);
            Scheduler.schedule(intake);
            s.step(10);
            check(!running(intake), "intake rejected while shooting");
            check(running(shoot), "shooting not interrupted");
            check(s.intake.power == RobotConfig.FEED_POWER || Storage.ballsShot == 1, "feed motor still the shooter's");
            check(s.runUntil(() -> !running(shoot), 3000), "shoot ends");
            check(Storage.ballsShot == 1, "ball shot");
            assertSafe(s, "end");
        });

        sim("shoot: scheduling a shot interrupts a running intake and takes over the motor", s -> {
            Command intake = RobotCommands.runIntake(s.robot, false);
            Scheduler.schedule(intake);
            s.run(50);
            check(s.intake.power > 0 && running(intake), "intake running");
            s.ballsLoaded = 1;
            Command shoot = RobotCommands.shoot(s.robot, 1, 1800);
            Scheduler.schedule(shoot);
            s.step(10);
            check(!running(intake), "intake interrupted by the higher-priority shot");
            check(running(shoot), "shot running");
            check(Storage.shootInProgress, "shoot state not clobbered by the intake's cleanup");
            check(s.runUntil(() -> !running(shoot), 8000), "shot completes");
            check(Storage.ballsShot == 1, "ball shot");
            assertSafe(s, "end");
        });

        sim("shoot: leaving the shooting zone before feeding holds the feed; re-entering resumes", s -> {
            RobotConfig.REQUIRE_LOCALIZATION = true;
            RobotConfig.SHOOT_ZONE_ENABLED = true;
            RobotConfig.SHOOT_ZONE_MIN_X = 0; RobotConfig.SHOOT_ZONE_MAX_X = 50;
            RobotConfig.SHOOT_ZONE_MIN_Y = 0; RobotConfig.SHOOT_ZONE_MAX_Y = 50;
            s.follower.pose = new Pose(100, 100, 0);   // outside
            s.ballsLoaded = 1;
            Command c = RobotCommands.shoot(s.robot, 1, 1800, 5000);
            Scheduler.schedule(c);
            s.run(1500);
            check(running(c), "still waiting for a window");
            check("WAIT_READY".equals(Storage.shootPhase), "phase: " + Storage.shootPhase);
            check(s.intake.power == 0 && s.stopper.position == RobotConfig.STOPPER_CLOSED, "nothing fed, stopper closed");
            check(Storage.flywheelReady, "flywheel itself is ready");
            s.follower.pose = new Pose(25, 25, 0);     // inside
            check(s.runUntil(() -> !running(c), 4000), "shoots once inside");
            check(Storage.ballsShot == 1, "shot");
        });

        sim("shoot: never entering the zone times out with the reason", s -> {
            RobotConfig.SHOOT_ZONE_ENABLED = true;
            RobotConfig.REQUIRE_LOCALIZATION = true;
            RobotConfig.SHOOT_ZONE_MIN_X = 0; RobotConfig.SHOOT_ZONE_MAX_X = 50;
            RobotConfig.SHOOT_ZONE_MIN_Y = 0; RobotConfig.SHOOT_ZONE_MAX_Y = 50;
            s.follower.pose = new Pose(100, 100, 0);
            Command c = RobotCommands.shoot(s.robot, 1, 1800, 700);
            Scheduler.schedule(c);
            check(s.runUntil(() -> !running(c), 4000), "ends");
            check(Storage.lastFailure.contains("outside shooting zone"), Storage.lastFailure);
            assertSafe(s, "timeout");
        });

        sim("shoot: stale or invalid localization prevents readiness", s -> {
            RobotConfig.REQUIRE_LOCALIZATION = true;
            s.ballsLoaded = 1;
            s.stampPose = false;   // follower.update() stamp stops arriving
            s.follower.pose = new Pose(10, 10, 0);
            Command c = RobotCommands.shoot(s.robot, 1, 1800, 800);
            Scheduler.schedule(c);
            check(s.runUntil(() -> !running(c), 4000), "ends");
            check(Storage.lastFailure.contains("localization"), Storage.lastFailure);
            check(s.shotsFired == 0, "no shot");
            Scheduler.reset(); Storage.invalidateHardwareState();
            s.stampPose = true;
            s.follower.pose = new Pose(Double.NaN, 0, 0);
            Command c2 = RobotCommands.shoot(s.robot, 1, 1800, 800);
            Scheduler.schedule(c2);
            check(s.runUntil(() -> !running(c2), 4000), "ends");
            check(Storage.lastFailure.contains("localization invalid"), Storage.lastFailure);
        });

        sim("shoot: readiness lost mid-ball does not abandon the ball; next ball waits for the window", s -> {
            RobotConfig.REQUIRE_LOCALIZATION = true;
            RobotConfig.SHOOT_ZONE_ENABLED = true;
            RobotConfig.SHOOT_ZONE_MIN_X = 0; RobotConfig.SHOOT_ZONE_MAX_X = 50;
            RobotConfig.SHOOT_ZONE_MIN_Y = 0; RobotConfig.SHOOT_ZONE_MAX_Y = 50;
            s.follower.pose = new Pose(25, 25, 0);
            s.ballsLoaded = 2;
            s.ballDelay = 0.4;
            Command c = RobotCommands.shoot(s.robot, 2, 1800, 6000);
            Scheduler.schedule(c);
            check(s.runUntil(() -> "FEED".equals(Storage.shootPhase), 8000), "feeding ball 1");
            s.follower.pose = new Pose(100, 100, 0);   // leave the zone mid-feed
            check(s.runUntil(() -> Storage.ballsShot == 1, 3000), "ball 1 still completes (no mid-feed abort)");
            s.run(1200);
            check(running(c) && "WAIT_READY".equals(Storage.shootPhase), "ball 2 waits: " + Storage.shootPhase);
            check(s.intake.power == 0 && s.shotsFired == 1, "no second feed while outside");
            s.follower.pose = new Pose(20, 20, 0);
            check(s.runUntil(() -> !running(c), 5000), "ball 2 after re-entering");
            check(Storage.ballsShot == 2, "both shot");
        });

    }
}
