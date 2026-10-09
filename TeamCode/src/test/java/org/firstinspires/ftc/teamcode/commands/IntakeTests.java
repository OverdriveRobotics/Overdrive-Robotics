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
public final class IntakeTests {
    private IntakeTests() {}

    public static void run() {
        sim("intake: indefinite when duration <= 0, timed otherwise, rejects NaN/inf/bad power, stops motor on cancel", s -> {
            Command forever = RobotCommands.runIntake(s.robot, false, 0.8, 0, null);
            Scheduler.schedule(forever);
            s.run(3000);
            check(running(forever) && s.intake.power == 0.8, "runs indefinitely for durationMs=0");
            Scheduler.cancel(forever);
            near(0, s.intake.power, 0, "stopped on cancel");
            Command neg = RobotCommands.runIntake(s.robot, true, 0.5, -1, null);
            Scheduler.schedule(neg);
            s.run(500);
            check(running(neg) && s.intake.power == -0.5, "negative duration also means indefinite; reverse power");
            Scheduler.cancel(neg);
            Command timed = RobotCommands.runIntake(s.robot, false, 1.0, 200, null);
            Scheduler.schedule(timed);
            check(s.runUntil(() -> !running(timed), 1000), "timed run ends");
            near(0, s.intake.power, 0, "stopped on completion");
            double[][] bad = {{Double.NaN, 100}, {2.0, 100}, {0, 100}, {-0.5, 100}, {1.0, Double.NaN}, {1.0, Double.POSITIVE_INFINITY}};
            for (double[] b : bad) {
                Command c = RobotCommands.runIntake(s.robot, false, b[0], b[1], null);
                Scheduler.schedule(c);
                s.step(10);
                check(!running(c), "rejected power=" + b[0] + " duration=" + b[1]);
                near(0, s.intake.power, 0, "motor not driven by rejected command");
            }
            Command stopWhen = RobotCommands.runIntake(s.robot, false, 1.0, 0, () -> s.nanos > 10_000_000_000L);
            Scheduler.schedule(stopWhen);
            check(s.runUntil(() -> !running(stopWhen), 20000), "stopWhen ends it");
            near(0, s.intake.power, 0, "stopped");
        });


        sim("intake: direction and power - forward is +power, reverse is -power, state follows the command", s -> {
            Command f = RobotCommands.runIntake(s.robot, false, 0.7, 0, null);
            Scheduler.schedule(f); s.run(50);
            near(0.7, s.intake.power, 1e-12, "forward");
            check(Storage.intakeState == Storage.IntakeState.INTAKING, "INTAKING");
            Scheduler.cancel(f);
            Command r = RobotCommands.runIntake(s.robot, true, 0.4, 0, null);
            Scheduler.schedule(r); s.run(50);
            near(-0.4, s.intake.power, 1e-12, "reverse");
            check(Storage.intakeState == Storage.IntakeState.REVERSING, "REVERSING");
            Scheduler.cancel(r);
            near(0, s.intake.power, 0, "stopped");
            check(Storage.intakeState == Storage.IntakeState.IDLE, "IDLE");
        });

        sim("intake: stopWhen ends it and completion stops the motor", s -> {
            final boolean[] go = {false};
            Command c = RobotCommands.runIntake(s.robot, false, 0.5, 0, () -> go[0]);
            Scheduler.schedule(c); s.run(100);
            check(running(c), "running");
            go[0] = true; s.run(30);
            check(!running(c) && s.intake.power == 0, "ended and stopped");
        });

        sim("intake: JAM DETECTION (exists, off by default) - high current for long enough stops the motor and reports JAMMED", s -> {
            RobotConfig.DETECT_INTAKE_JAM = true;
            RobotConfig.INTAKE_JAM_AMPS = 5; RobotConfig.INTAKE_JAM_MS = 200;
            s.intake.currentAmps = 3;   // normal load
            Command ok = RobotCommands.runIntake(s.robot, false, 0.8, 0, null);
            Scheduler.schedule(ok); s.run(600);
            check(running(ok) && s.intake.power == 0.8, "normal current: keeps running");
            s.intake.currentAmps = 8;   // jam
            s.run(150);
            check(running(ok), "a short spike below INTAKE_JAM_MS is tolerated");
            s.run(150);
            check(!running(ok) && s.intake.power == 0, "jam: stopped");
            check(Storage.intakeState == Storage.IntakeState.JAMMED && Storage.lastFailure.contains("jam"), "reported: " + Storage.lastFailure);
            // a spike that clears resets the timer
            Storage.invalidateHardwareState(); RobotConfig.DETECT_INTAKE_JAM = true;
            s.intake.currentAmps = 3;
            Command c2 = RobotCommands.runIntake(s.robot, false, 0.8, 0, null);
            Scheduler.schedule(c2); s.run(50);
            for (int i = 0; i < 4; i++) { s.intake.currentAmps = 8; s.run(150); s.intake.currentAmps = 3; s.run(50); }
            check(running(c2), "intermittent spikes never accumulate into a jam");
            Scheduler.cancel(c2);
        });

        sim("stopper: only the configured OPEN/CLOSED positions are ever commanded, open before feed, closed after", s -> {
            RobotConfig.STOPPER_OPEN = 0.71; RobotConfig.STOPPER_CLOSED = 0.12;
            final java.util.Set<Double> seen = new java.util.HashSet<>();
            s.ballsLoaded = 2;
            Command c = RobotCommands.shoot(s.robot, 2, 1800);
            Scheduler.schedule(c);
            for (int i = 0; i < 3000 && running(c); i++) {
                s.step(10);
                if (!Double.isNaN(s.stopper.position)) seen.add(s.stopper.position);
                if (s.intake.power > 0) check(s.stopper.position == 0.71, "feed only while the stopper command is OPEN");
            }
            check(seen.equals(new java.util.HashSet<>(java.util.Arrays.asList(0.71, 0.12))), "positions commanded: " + seen);
            near(0.12, s.stopper.position, 0, "closed at the end");
        });
    }
}
