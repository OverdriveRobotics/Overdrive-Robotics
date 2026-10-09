package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.info;
import static org.firstinspires.ftc.teamcode.T.near;
import static org.firstinspires.ftc.teamcode.T.test;

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

/** Integration tests: real Ivy scheduler, simulated hardware/physics, fake clock. SIMULATION ONLY. */
public final class CommandTests {
    private CommandTests() {}

    private interface Body { void run(SimRobot s) throws Exception; }

    private static void sim(String name, Body b) {
        test(name, () -> {
            SimRobot s = SimRobot.fresh();
            try { b.run(s); } finally { s.cleanup(); }
        });
    }

    private static boolean running(Command c) { return Scheduler.isScheduled(c); }

    private static Command spinUpReady(SimRobot s, double target) {
        Command c = RobotCommands.spinUpFlywheel(s.robot, target);
        Scheduler.schedule(c);
        check(s.runUntil(() -> !running(c), 6000), "spin-up to " + target + " should finish");
        return c;
    }

    private static void assertSafe(SimRobot s, String why) {
        check(s.intake.power == 0, why + ": feed motor stopped");
        near(RobotConfig.STOPPER_CLOSED, s.stopper.position, 0, why + ": stopper closed");
        check(!Storage.shootInProgress, why + ": shootInProgress cleared");
    }

    public static void run() {
        // ============================================================ flywheel
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

        // ============================================================ shooting
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

        // ============================================================ validation / missing hardware
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

        // ============================================================ targets + turret
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

        // ============================================================ composition / transitions
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

        System.out.println();
    }
}
