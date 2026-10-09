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
public final class LifecycleTests {
    private LifecycleTests() {}

    public static void run() {
        sim("OpMode transition: Scheduler.reset() alone does NOT clean hardware; safeShutdown + invalidate does", s -> {
            s.ballsLoaded = 1;
            s.ballDelay = 5;
            Command c = RobotCommands.shoot(s.robot, 1, 1800);
            Scheduler.schedule(c);
            check(s.runUntil(() -> "FEED".equals(Storage.shootPhase), 8000), "feeding");
            Scheduler.reset();
            check(s.intake.power > 0, "documented behaviour: reset() leaves the feed motor running");
            s.robot.safeShutdown(false);
            assertSafe(s, "after safeShutdown(false)");
            near(1800, Storage.flywheelTargetVelocity, 0, "flywheel policy: kept");
            s.robot.safeShutdown(true);
            near(0, Storage.flywheelTargetVelocity, 0, "flywheel stopped when requested");
            near(0, s.shooterL.velCmd, 0, "motors told to stop");
            Storage.setTarget(1, 3, 4);
            Storage.invalidateHardwareState();
            check(!Storage.flywheelReady && Storage.flywheelFault == null && Storage.lastFailure == null && !Storage.shootInProgress,
                    "no stale readiness / fault / shooting state for the next OpMode");
            check(Storage.targetB.x == 3, "targets are configuration and survive");
        });

        sim("scheduler: a second shoot command cannot displace a running one; one regulator only", s -> {
            s.ballsLoaded = 2;
            Command a = RobotCommands.shoot(s.robot, 2, 1800);
            Scheduler.schedule(a);
            s.run(100);
            Command b = RobotCommands.shoot(s.robot, 1, 1800);
            Scheduler.schedule(b);
            s.step(10);
            check(running(a) && !running(b), "first keeps the resources (ConflictBehavior.CANCEL)");
            RobotCommands.ensureFlywheelRegulator(s.robot);
            RobotCommands.ensureFlywheelRegulator(s.robot);
            check(s.runUntil(() -> !running(a), 10000), "first completes");
            check(Storage.ballsShot == 2, "two shots by the first command");
        });

    }
}
