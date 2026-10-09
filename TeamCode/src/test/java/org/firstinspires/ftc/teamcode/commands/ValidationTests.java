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
public final class ValidationTests {
    private ValidationTests() {}

    public static void run() {
        sim("validation: null robot and missing hardware never throw; commands fail safely", s -> {
            Command[] cs = {
                    RobotCommands.shoot(null, 1, 1800), RobotCommands.spinUpFlywheel(null, 1800),
                    RobotCommands.stopFlywheel(null), RobotCommands.runIntake(null, false),
                    RobotCommands.aimTurret(null)};
            for (Command c : cs) Scheduler.schedule(c);
            s.run(100);
            for (Command c : cs) check(!running(c) || c == cs[4], "command ended or inert");
            RobotHardware partial = new RobotHardware();   // all fields null
            Scheduler.schedule(RobotCommands.shoot(partial, 1, 1800), RobotCommands.spinUpFlywheel(partial, 1800),
                    RobotCommands.runIntake(partial, true));
            s.run(100);
            check(Storage.lastFailure != null, "failures were reported");
        });

        sim("validation: invalid parameters are rejected before any hardware is touched", s -> {
            double[] badVel = {Double.NaN, -5, 0, Double.POSITIVE_INFINITY, RobotConfig.MAX_FLYWHEEL_VELOCITY + 1};
            for (double v : badVel) {
                Command c = RobotCommands.spinUpFlywheel(s.robot, v);
                Scheduler.schedule(c);
                s.step(10);
                check(!running(c), "spin-up rejected " + v);
                check(Storage.flywheelTargetVelocity == 0, "target untouched by " + v);
                Command sh = RobotCommands.shoot(s.robot, 1, v);
                Scheduler.schedule(sh);
                s.step(10);
                check(!running(sh), "shoot rejected velocity " + v);
            }
            int[] badBalls = {0, -1, RobotConfig.MAX_BALLS + 1};
            for (int n : badBalls) {
                Command sh = RobotCommands.shoot(s.robot, n);
                Scheduler.schedule(sh);
                s.step(10);
                check(!running(sh), "ball count rejected " + n);
            }
            Command tol = RobotCommands.spinUpFlywheel(s.robot, 1800, 0, 100, 1000);
            Scheduler.schedule(tol);
            s.step(10);
            check(!running(tol), "tolerance 0 rejected");
            Command tol2 = RobotCommands.spinUpFlywheel(s.robot, 1800, Double.NaN, 100, 1000);
            Scheduler.schedule(tol2);
            s.step(10);
            check(!running(tol2), "NaN tolerance rejected");
            check(Double.isNaN(s.stopper.position) && s.stopper.writes == 0, "stopper never moved");
            check(s.intake.powerWrites == 0 && s.shooterL.powerWrites == 0 && s.shooterL.velCalls == 0, "no motor writes");
        });

    }
}
