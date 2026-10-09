package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.near;
import static org.firstinspires.ftc.teamcode.commands.SimSupport.*;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.hardware.Storage;

import java.io.File;
import java.nio.file.Files;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/** Shared robot state: staleness, fault transitions, counters, re-initialisation, sole-writer ownership. */
public final class StateSafetyTests {
    private StateSafetyTests() {}

    public static void run() {
        sim("state: readiness goes stale on its own when nothing refreshes it (no stuck-true flag)", s -> {
            spinUpReady(s, 1800);
            s.run(100);
            check(Storage.isFlywheelReadyFresh(), "fresh while the regulator runs");
            Scheduler.reset();            // regulator gone: nobody updates the measurement any more
            s.nanos += 300_000_000L;      // 300 ms later
            check(!Storage.isFlywheelReadyFresh(), "stale sample -> not ready even though the boolean is still true");
            check(Storage.flywheelReady, "(the raw flag is stale-true: that is exactly why consumers use the fresh check)");
        });

        sim("state: fault transitions - set by a failure, cleared only by a new request, preserved by Stop", s -> {
            s.flywheelDead = true;
            Command c = RobotCommands.spinUpFlywheel(s.robot, 1800, 50, 150, 600);
            Scheduler.schedule(c);
            check(s.runUntil(() -> !running(c), 3000), "times out");
            check(Storage.flywheelFault != null, "fault latched");
            Scheduler.schedule(RobotCommands.stopFlywheel(s.robot));
            s.step(10);
            check(Storage.flywheelFault != null, "stop preserves diagnostics");
            near(0, Storage.flywheelTargetVelocity, 0, "target cleared");
            s.flywheelDead = false;
            Command again = RobotCommands.spinUpFlywheel(s.robot, 1800);
            Scheduler.schedule(again);
            s.step(10);
            check(Storage.flywheelFault == null, "a new request clears the fault");
            check(s.runUntil(() -> !running(again), 6000), "and spin-up works again after the fault");
        });

        sim("state: shot counters - per-command count resets, total accumulates, failures do not count", s -> {
            s.ballsLoaded = 5;
            Command a = RobotCommands.shoot(s.robot, 2, 1800);
            Scheduler.schedule(a);
            check(s.runUntil(() -> !running(a), 15000), "first");
            check(Storage.ballsShot == 2 && Storage.totalBallsShot == 2, "2/2");
            Command b = RobotCommands.shoot(s.robot, 1, 1800);
            Scheduler.schedule(b);
            check(s.runUntil(() -> !running(b), 15000), "second");
            check(Storage.ballsShot == 1 && Storage.totalBallsShot == 3, "per-command reset, total accumulates: " + Storage.ballsShot + "/" + Storage.totalBallsShot);
            s.ballsLoaded = 0;
            Command c = RobotCommands.shoot(s.robot, 1, 1800);
            Scheduler.schedule(c);
            check(s.runUntil(() -> !running(c), 15000), "third (no ball)");
            check(Storage.totalBallsShot == 3, "a failed shot is not counted");
        });

        sim("state: re-initialisation clears hardware-derived state but keeps configuration", s -> {
            spinUpReady(s, 1800);
            Storage.setTarget(1, 5, 6); Storage.selectTarget(1);
            Storage.lastFailure = "x"; Storage.readyToShoot = true; Storage.turretAngle = 1.0;
            Storage.invalidateHardwareState();
            check(Storage.flywheelTargetVelocity == 0 && !Storage.flywheelReady && Storage.flywheelFault == null && Storage.lastFailure == null, "flywheel + failure state cleared");
            check(!Storage.readyToShoot && Double.isNaN(Storage.turretAngle) && !Storage.shootInProgress, "readiness + turret + shot state cleared");
            check(Storage.targetB.x == 5 && Storage.activeTargetIndex == 1, "targets and selection are configuration and survive");
            check(!Storage.isFlywheelReadyFresh(), "nothing counts as ready after init");
        });

        sim("state: target switching mid-run changes the reference and readiness immediately", s -> {
            org.firstinspires.ftc.teamcode.control.TurretConfig.HARDWARE_CONFIGURED = true;
            RobotConfig.REQUIRE_TURRET_ALIGNED = true;
            Storage.setTarget(0, 10, 10); Storage.setTarget(1, 10, -10);
            Scheduler.schedule(RobotCommands.aimTurret(s.robot), RobotCommands.readinessMonitor(s.robot));
            RobotCommands.ensureFlywheelRegulator(s.robot);
            Scheduler.schedule(RobotCommands.spinUpFlywheel(s.robot, 1800));
            s.run(4000);
            check(Storage.readyToShoot, "ready on target A: " + Storage.notReadyReason);
            Storage.selectTarget(1);
            s.run(60);
            check(!Storage.readyToShoot && Storage.notReadyReason.contains("turret not aligned"), "switch invalidates readiness at once: " + Storage.notReadyReason);
            s.run(3500);
            check(Storage.readyToShoot, "ready again on target B: " + Storage.notReadyReason);
        });

        sim("ownership: only the regulator, Stop and the explicit shutdown ever write the shooter motors", s -> {
            s.ballsLoaded = 3;
            RobotCommands.startShooterSystems(s.robot);
            Command c = RobotCommands.shoot(s.robot, 3, 1800);
            Scheduler.schedule(c);
            check(s.runUntil(() -> !running(c), 20000), "three-ball shot");
            Scheduler.schedule(RobotCommands.stopFlywheel(s.robot));
            s.run(100);
            s.robot.safeShutdown(true);
            java.util.List<String> writers = new java.util.ArrayList<>(s.shooterL.writers);
            writers.addAll(s.shooterR.writers);
            java.util.Set<String> allowed = new java.util.HashSet<>(java.util.Arrays.asList("FlywheelRegulator", "StopFlywheel", "RobotHardware"));
            for (String w : writers) check(allowed.contains(w), "unexpected shooter-motor writer: " + w + " in " + writers);
            check(writers.contains("FlywheelRegulator"), "the regulator is the continuous writer: " + writers);
        });

        sim("ownership (source check): only FlywheelRegulator ever sets flywheelReady to a non-false value", s -> {
            File root = new File("TeamCode/src/main/java/org/firstinspires/ftc/teamcode");
            Pattern p = Pattern.compile("flywheelReady\\s*=\\s*([^;]+);");
            java.util.List<String> offenders = new java.util.ArrayList<>();
            for (File f : listJava(root)) {
                String src = new String(Files.readAllBytes(f.toPath()));
                Matcher m = p.matcher(src);
                while (m.find()) {
                    String rhs = m.group(1).trim();
                    if (!rhs.equals("false") && !f.getName().equals("FlywheelRegulator.java")) offenders.add(f.getName() + ": " + m.group(0));
                }
            }
            check(offenders.isEmpty(), "readiness set outside the regulator: " + offenders);
        });
    }

    static java.util.List<File> listJava(File dir) {
        java.util.List<File> out = new java.util.ArrayList<>();
        File[] fs = dir.listFiles();
        if (fs == null) return out;
        for (File f : fs) { if (f.isDirectory()) out.addAll(listJava(f)); else if (f.getName().endsWith(".java")) out.add(f); }
        return out;
    }
}
