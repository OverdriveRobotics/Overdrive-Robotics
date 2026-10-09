package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.near;
import static org.firstinspires.ftc.teamcode.commands.SimSupport.*;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.hardware.Storage;

/** Cancellation in EVERY ShootBalls phase leaves the mechanism safe and the flywheel target untouched. */
public final class ShootingCancelTests {
    private ShootingCancelTests() {}

    public static void run() {
        for (final String phase : new String[]{"WAIT_READY", "OPENING", "FEED", "CLOSING", "RECOVER"}) {
            sim("shoot: cancel while in " + phase + " is safe", s -> {
                s.ballsLoaded = 3;
                if (phase.equals("FEED")) s.ballDelay = 5;
                Command c = RobotCommands.shoot(s.robot, 3, 1800);
                Scheduler.schedule(c);
                if (phase.equals("WAIT_READY")) s.run(50);   // flywheel still spinning up
                else check(s.runUntil(() -> phase.equals(Storage.shootPhase), 20000), "reached " + phase);
                check(phase.equals(Storage.shootPhase), "in " + phase + " (was " + Storage.shootPhase + ")");
                Scheduler.cancel(c);
                assertSafe(s, "cancel in " + phase);
                near(1800, Storage.flywheelTargetVelocity, 0, "flywheel target untouched");
                check(Storage.lastFailure != null && Storage.lastFailure.contains("interrupted"), "interruption reported: " + Storage.lastFailure);
                s.run(200);
                check(s.intake.power == 0 && s.stopper.position == RobotConfig.STOPPER_CLOSED, "stays safe afterwards");
            });
        }

        sim("shoot: a ball already being fed is finished (safe-completion policy) and no further ball starts without readiness", s -> {
            RobotConfig.REQUIRE_LOCALIZATION = true;
            RobotConfig.SHOOT_ZONE_ENABLED = true;
            RobotConfig.SHOOT_ZONE_MIN_X = 0; RobotConfig.SHOOT_ZONE_MAX_X = 50;
            RobotConfig.SHOOT_ZONE_MIN_Y = 0; RobotConfig.SHOOT_ZONE_MAX_Y = 50;
            s.follower.pose = new com.pedropathing.math.Pose(25, 25, 0);
            s.ballsLoaded = 3; s.ballDelay = 0.5;
            Command c = RobotCommands.shoot(s.robot, 3, 1800, 8000);
            Scheduler.schedule(c);
            check(s.runUntil(() -> "FEED".equals(Storage.shootPhase), 8000), "feeding ball 1");
            s.follower.pose = new com.pedropathing.math.Pose(90, 90, 0);   // leave the window mid-ball
            check(s.runUntil(() -> Storage.ballsShot == 1, 3000), "ball 1 completes");
            int shotsAtExit = s.shotsFired;
            s.run(2500);
            check(s.shotsFired == shotsAtExit && s.intake.power == 0, "no further ball while outside the window");
            Scheduler.cancel(c);
            assertSafe(s, "end");
        });
    }
}
