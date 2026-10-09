package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.near;
import static org.firstinspires.ftc.teamcode.commands.SimSupport.*;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.line;

import org.firstinspires.ftc.teamcode.hardware.Storage;

/** Autonomous building blocks (AutoRoutines) composed from production commands, driven by the stub follower. */
public final class AutoRoutineTests {
    private AutoRoutineTests() {}

    private static Path path(double x0, double x1) { return line(new Pose(x0, 0, 0), new Pose(x1, 0, 0)).constant(0); }

    public static void run() {
        sim("auto: shootWhileDriving shoots during the path, then holds the final pose", s -> {
            s.follower.followDurationLoops = 400;
            s.ballsLoaded = 3;
            RobotCommands.startShooterSystems(s.robot);
            Command c = AutoRoutines.shootWhileDriving(s.robot, path(0, 40), 3, 1800, 6000);
            Scheduler.schedule(c);
            int shotsBeforePathEnd = -1;
            for (int i = 0; i < 2500 && running(c); i++) {
                s.step(10);
                if (i == 390) shotsBeforePathEnd = Storage.ballsShot;
            }
            check(shotsBeforePathEnd >= 1, "shooting began before the path finished: " + shotsBeforePathEnd);
            check(Storage.ballsShot == 3 && !running(c), "all shot and the routine ended");
            check(s.follower.holdCalls == 1, "final pose held exactly once");
            assertSafe(s, "end");
        });

        sim("auto: cycle runs the intake only while driving to the pickup; its end() stops the motor when the path ends", s -> {
            s.follower.followDurationLoops = 200;
            s.ballsLoaded = 2;
            RobotCommands.startShooterSystems(s.robot);
            Command c = AutoRoutines.cycle(s.robot, path(40, 0), path(0, 40), 2, 1800, 6000);
            Scheduler.schedule(c);
            s.run(500);   // mid-pickup
            check(s.intake.power > 0 && Storage.intakeState == Storage.IntakeState.INTAKING, "intake running during the pickup path");
            check(s.runUntil(() -> s.follower.followCalls >= 2, 3000), "second path (back to shoot) started");
            check(s.intake.power == 0 || Storage.shootInProgress, "intake stopped when the pickup path finished");
            check(s.runUntil(() -> !running(c), 15000), "cycle completes");
            check(Storage.ballsShot == 2, "balls shot on the way back");
            near(0, s.intake.power, 0, "feed off at the end");
        });

        sim("auto: cancelling a routine mid-way leaves the mechanisms safe and the flywheel regulator alive", s -> {
            s.follower.followDurationLoops = 1000;
            s.ballsLoaded = 3;
            RobotCommands.startShooterSystems(s.robot);
            Command c = AutoRoutines.cycle(s.robot, path(40, 0), path(0, 40), 3, 1800, 6000);
            Scheduler.schedule(c);
            s.run(600);
            Scheduler.cancel(c);
            near(0, s.intake.power, 0, "intake stopped by its end()");
            check(Double.isNaN(s.stopper.position) || s.stopper.position == RobotConfig.STOPPER_CLOSED, "stopper never opened (untouched or closed)");
            check(!Storage.shootInProgress, "no shot state left behind");
        });

        sim("auto: sequencing - consecutive routines run strictly one after the other", s -> {
            s.follower.followDurationLoops = 150;
            s.ballsLoaded = 2;
            RobotCommands.startShooterSystems(s.robot);
            Command both = com.pedropathing.ivy.groups.Groups.sequential(
                    AutoRoutines.shootWhileDriving(s.robot, path(0, 30), 1, 1800, 6000),
                    AutoRoutines.shootWhileDriving(s.robot, path(30, 60), 1, 1800, 6000));
            Scheduler.schedule(both);
            check(s.runUntil(() -> !running(both), 25000), "both done");
            check(Storage.totalBallsShot == 2 && s.follower.followCalls == 2 && s.follower.holdCalls == 2, "two paths, two holds, two balls");
        });
    }
}
