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
public final class FlywheelRegulatorTests {
    private FlywheelRegulatorTests() {}

    public static void run() {
        sim("flywheel: readiness is not declared before the stability window; spin-up ends once stable", s -> {
            Command c = RobotCommands.spinUpFlywheel(s.robot, 1800);
            Scheduler.schedule(c);
            int firstInBand = -1, firstReady = -1;
            for (int i = 0; i < 1000 && running(c); i++) {
                s.step(10);
                if (firstInBand < 0 && Math.abs(s.omega - 1800) <= Storage.flywheelTolerance) firstInBand = i;
                if (firstReady < 0 && Storage.flywheelReady) firstReady = i;
            }
            info("in band at step " + firstInBand + ", ready at step " + firstReady);
            check(firstInBand >= 0 && firstReady >= 0, "both happen");
            check((firstReady - firstInBand) * 10 >= RobotConfig.FLYWHEEL_STABLE_MS - 20, "ready only after the stability window");
            check(!running(c), "command ended");
            near(1800, Storage.flywheelTargetVelocity, 0, "target stays commanded after the command ends");
            s.run(200);
            check(Storage.flywheelReady, "still ready after spin-up command finished");
        });

        sim("flywheel: a ball-induced velocity drop invalidates readiness on the very next loop", s -> {
            spinUpReady(s, 1800);
            s.run(100);
            check(Storage.flywheelReady, "ready before");
            s.omega -= 300;
            s.step(10);
            check(!Storage.flywheelReady, "ready must drop immediately");
            check(!Storage.isFlywheelReadyFresh(), "fresh check also false");
        });

        sim("flywheel: repeating the same target changes nothing (no motor re-command, no readiness reset)", s -> {
            spinUpReady(s, 1800);
            s.run(100);
            int calls = s.shooterL.velCalls;
            double stableBefore = Storage.flywheelStableForMs;
            Command again = RobotCommands.spinUpFlywheel(s.robot, 1800);
            Scheduler.schedule(again);
            s.step(10);
            check(Storage.flywheelReady, "readiness not reset");
            check(s.shooterL.velCalls == calls, "no extra setVelocity");
            check(Storage.flywheelStableForMs >= stableBefore, "stability timer not reset");
            s.step(10);
            check(!running(again), "repeat finishes immediately");
        });

        sim("flywheel: a new target replaces the old one deterministically; the superseded spin-up is not a failure", s -> {
            Command first = RobotCommands.spinUpFlywheel(s.robot, 1800);
            Scheduler.schedule(first);
            s.run(100);
            Command second = RobotCommands.spinUpFlywheel(s.robot, 2200);
            Scheduler.schedule(second);
            s.step(10);
            near(2200, Storage.flywheelTargetVelocity, 0, "latest request wins");
            check(!Storage.flywheelReady, "readiness reset for the new target");
            s.step(10);
            check(!running(first), "superseded command ended");
            check(Storage.lastFailure == null, "no failure reported: " + Storage.lastFailure);
            check(s.runUntil(() -> !running(second), 6000), "new target reached");
            near(2200, s.shooterL.velCmd, 0, "motors were commanded to the new target");
        });

        sim("flywheel: stop clears target + readiness, stops both motors, keeps fault diagnostics", s -> {
            spinUpReady(s, 1800);
            Storage.flywheelFault = "test fault";
            Scheduler.schedule(RobotCommands.stopFlywheel(s.robot));
            s.step(10);
            near(0, Storage.flywheelTargetVelocity, 0, "target cleared");
            check(!Storage.flywheelReady, "not ready");
            near(0, s.shooterL.velCmd, 0, "left stopped");
            near(0, s.shooterR.velCmd, 0, "right stopped");
            near(0, s.shooterL.power, 0, "left power");
            near(0, s.shooterR.power, 0, "right power");
            check("test fault".equals(Storage.flywheelFault), "fault retained");
            s.run(2000);
            check(s.omega < 50, "spun down: " + s.omega);
        });

        sim("flywheel: unreachable target times out, latches a fault and never reports ready", s -> {
            s.flywheelDead = true;
            Command c = RobotCommands.spinUpFlywheel(s.robot, 1800, 50, 150, 800);
            Scheduler.schedule(c);
            check(s.runUntil(() -> !running(c), 3000), "ends");
            check(Storage.flywheelFault != null && Storage.flywheelFault.contains("did not reach"), "fault: " + Storage.flywheelFault);
            check(!Storage.flywheelReady, "not ready");
            check(Storage.lastFailure != null, "failure reported");
        });

        sim("flywheel: NaN velocity reading faults the flywheel and zeroes its output", s -> {
            spinUpReady(s, 1800);
            s.nanVelocity = true;
            s.run(50);
            check(Storage.flywheelFault != null && Storage.flywheelFault.contains("invalid"), "fault: " + Storage.flywheelFault);
            check(!Storage.flywheelReady, "not ready");
            near(0, s.shooterL.power, 0, "power zero");
        });

        sim("flywheel LQR path: reaches target under voltage control, power in [0,1], battery sag handled, fault -> 0", s -> {
            FlywheelConfig.MODEL_IDENTIFIED = true;
            s.vbat = 10.5;
            double maxP = 0, minP = 1;
            Command c = RobotCommands.spinUpFlywheel(s.robot, 1800);
            Scheduler.schedule(c);
            for (int i = 0; i < 1500 && running(c); i++) {
                s.step(10);
                maxP = Math.max(maxP, s.shooterL.power);
                minP = Math.min(minP, s.shooterL.power);
            }
            check(!running(c), "spin-up finished under LQR");
            check(s.shooterL.mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER, "raw power mode");
            check(maxP <= 1.0 && minP >= 0.0, "power within [0,1]: " + minP + ".." + maxP);
            s.run(1000);
            near(1800, s.omega, 20, "holds target");
            info("LQR flywheel power at hold = " + s.shooterL.power + " (vbat " + s.vbat + ")");
            s.omega -= 300;
            s.step(10);
            check(!Storage.flywheelReady, "drop invalidates readiness (LQR path)");
            s.nanVelocity = true;
            s.run(50);
            near(0, s.shooterL.power, 0, "fault -> zero power");
            check(Storage.flywheelFault != null, "fault set");
        });


        sim("flywheel measurement: the two shooter motors are averaged and the sample is timestamped with the robot clock", s -> {
            s.rightOffset = 40;   // right reads 40 ticks/s higher than left
            spinUpReady(s, 1800);
            s.run(100);
            near(1800 + 20, Storage.flywheelMeasuredVelocity, 25, "average of left and right");
            long age = s.nanos - Storage.flywheelMeasuredNanos;
            check(age >= 0 && age <= 20_000_000L, "timestamp comes from RobotClock and is current: " + age + " ns");
            s.step(10);
            check(Storage.flywheelMeasuredNanos <= s.nanos, "never in the future");
        });

        sim("flywheel measurement: one motor reporting NaN invalidates the whole reading (no half-average)", s -> {
            spinUpReady(s, 1800);
            s.nanRightOnly = true;
            s.run(50);
            check(Storage.flywheelFault != null && Storage.flywheelFault.contains("invalid"), "fault: " + Storage.flywheelFault);
            check(!Storage.flywheelReady && s.shooterL.power == 0, "not ready, output zero");
            s.nanRightOnly = false;
            Command again = RobotCommands.spinUpFlywheel(s.robot, 1800);
            Scheduler.schedule(again);
            check(s.runUntil(() -> !running(again), 8000) && Storage.flywheelFault == null, "recovers after a fresh request");
        });

        sim("flywheel measurement: a stale sample (regulator stopped) is not trusted for readiness", s -> {
            spinUpReady(s, 1800);
            s.run(100);
            Scheduler.reset();
            s.nanos += 250_000_000L;
            check(!Storage.isFlywheelReadyFresh(), "250 ms old sample rejected");
        });
    }
}
