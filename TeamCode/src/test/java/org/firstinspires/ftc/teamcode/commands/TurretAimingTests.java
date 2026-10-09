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
public final class TurretAimingTests {
    private TurretAimingTests() {}

    public static void run() {
        sim("targets: two stored points, runtime switching, validation", s -> {
            check(Storage.setTarget(0, 10, 20) && Storage.setTarget(1, -5, 7), "set both");
            check(!Storage.setTarget(2, 1, 1) && !Storage.setTarget(-1, 1, 1), "bad index rejected");
            check(!Storage.setTarget(0, Double.NaN, 1) && !Storage.setTarget(0, 1, Double.POSITIVE_INFINITY), "bad coords rejected");
            near(10, Storage.targetA.x, 0, "A unchanged by rejected writes");
            check(Storage.selectTarget(1) && Storage.activeTarget().x == -5, "select B");
            check(!Storage.selectTarget(5) && Storage.activeTargetIndex == 1, "invalid selection leaves it unchanged");
            Storage.toggleTarget();
            check(Storage.activeTargetIndex == 0 && Storage.activeTarget().y == 20, "toggle back to A");
            Storage.invalidateHardwareState();
            check(Storage.targetA.x == 10 && Storage.activeTargetIndex == 0, "targets survive an OpMode transition");
        });

        sim("turret: AimTurret tracks the active target, re-aims on a target switch, rejects unreachable/invalid targets", s -> {
            TurretConfig.HARDWARE_CONFIGURED = true;
            Storage.setTarget(0, 10, 10);     // +45 deg from the origin
            Storage.setTarget(1, 10, -10);    // -45 deg
            s.follower.pose = new Pose(0, 0, 0);
            Command aim = RobotCommands.aimTurret(s.robot);
            Scheduler.schedule(aim);
            s.run(2000);
            near(Math.toRadians(45), s.theta, Math.toRadians(1), "turret at +45 deg");
            near(Math.toRadians(45), Storage.turretDesiredAngle, 1e-9, "desired published");
            check(Storage.turretFault == null, "no fault: " + Storage.turretFault);
            check(Math.abs(s.turretM.power) <= 1, "power within limits");

            Storage.selectTarget(1);          // runtime switch, no restart, command keeps running
            check(running(aim), "same command keeps running");
            s.run(2500);
            near(Math.toRadians(-45), s.theta, Math.toRadians(1), "turret re-aimed to -45 deg");

            Storage.setTarget(1, -10, 10);    // +135 deg: outside +-90 deg mechanical range
            s.run(500);
            check(!Storage.turretReachable, "unreachable target flagged");
            check(Math.abs(s.theta) < Math.toRadians(100), "turret did not run past its limits");

            Storage.selectTarget(0);
            s.run(2500);
            near(Math.toRadians(45), s.theta, Math.toRadians(1), "back at A");

            Storage.targetA = new Storage.FieldPoint(Double.NaN, 3);
            s.run(100);
            check(Storage.turretFault != null && Storage.turretFault.contains("invalid"), "invalid target fault: " + Storage.turretFault);
            near(0, s.turretM.power, 0, "no output on an invalid target");
            Scheduler.cancel(aim);
            near(0, s.turretM.power, 0, "stopped on cancel");
        });

        sim("turret: follows a target while the robot drives and turns", s -> {
            TurretConfig.HARDWARE_CONFIGURED = true;
            Storage.setTarget(0, 60, 0);
            s.follower.pose = new Pose(0, -20, 0);
            Command aim = RobotCommands.aimTurret(s.robot);
            Scheduler.schedule(aim);
            double maxErr = 0;
            for (int i = 0; i < 300; i++) {     // 3 s: drive +x at 10 in/s, turn 0.3 rad/s
                double x = 10 * i * 0.01, h = 0.3 * i * 0.01;
                s.follower.pose = new Pose(x, -20, h);
                s.follower.vel = new Velocity(10, 0, 0.3);
                s.step(10);
                if (i > 150) maxErr = Math.max(maxErr, Math.abs(Storage.turretError));
            }
            info("max |aim error| while driving+turning (after 1.5 s): " + Math.toDegrees(maxErr) + " deg");
            check(maxErr < Math.toRadians(4), "error stays small");
        });

        sim("turret: uncalibrated hardware is never powered but still reports geometry", s -> {
            Storage.setTarget(0, 10, 10);
            Command aim = RobotCommands.aimTurret(s.robot);
            Scheduler.schedule(aim);
            s.run(300);
            near(0, s.turretM.power, 0, "motor held off");
            check(Storage.turretFault != null && Storage.turretFault.contains("not calibrated"), "reason: " + Storage.turretFault);
            near(Math.toRadians(45), Storage.turretDesiredAngle, 1e-9, "desired angle still computed");
        });

        sim("turret: stale pose cuts output; readiness requires the turret aligned (when enabled)", s -> {
            TurretConfig.HARDWARE_CONFIGURED = true;
            RobotConfig.REQUIRE_TURRET_ALIGNED = true;
            Storage.setTarget(0, 10, 10);
            s.follower.pose = new Pose(0, 0, 0);
            s.ballsLoaded = 1;
            Scheduler.schedule(RobotCommands.aimTurret(s.robot));
            Command sh = RobotCommands.shoot(s.robot, 1, 1800, 6000);
            Scheduler.schedule(sh);
            // Turret starts 45 deg away: must not feed until it is aligned.
            boolean fedWhileMisaligned = false;
            for (int i = 0; i < 1500 && running(sh); i++) {
                s.step(10);
                if (s.intake.power > 0 && Math.abs(Storage.turretError) > Math.toRadians(TurretConfig.ALIGN_TOLERANCE_RAD * 57.3 + 1))
                    fedWhileMisaligned = true;
            }
            check(!fedWhileMisaligned, "never fed while misaligned");
            check(Storage.ballsShot == 1, "shot once aligned: " + Storage.lastFailure);
            // stale pose -> output 0 and a fault
            Scheduler.reset(); Storage.invalidateHardwareState();
            s.stampPose = false;
            Scheduler.schedule(RobotCommands.aimTurret(s.robot));
            s.run(300);
            near(0, s.turretM.power, 0, "stale pose -> no output");
            check(Storage.turretFault.contains("stale"), Storage.turretFault);
        });

    }
}
