package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.info;
import static org.firstinspires.ftc.teamcode.T.near;
import static org.firstinspires.ftc.teamcode.commands.SimSupport.*;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.calibration.MovingShotProcedures;
import org.firstinspires.ftc.teamcode.calibration.ProcedureResult;
import org.firstinspires.ftc.teamcode.calibration.ShootWindowProcedures;
import org.firstinspires.ftc.teamcode.control.TurretConfig;
import org.firstinspires.ftc.teamcode.hardware.Storage;

/** Shooting-window and staged moving-shot procedures against the simulator. SIMULATION ONLY. */
public final class WindowMovingShotCalibrationTests {
    private WindowMovingShotCalibrationTests() {}

    private static void zone() {
        RobotConfig.REQUIRE_LOCALIZATION = true;
        RobotConfig.SHOOT_ZONE_ENABLED = true;
        RobotConfig.SHOOT_ZONE_MIN_X = 0; RobotConfig.SHOOT_ZONE_MAX_X = 50;
        RobotConfig.SHOOT_ZONE_MIN_Y = 0; RobotConfig.SHOOT_ZONE_MAX_Y = 50;
    }

    public static void run() {
        sim("cal window monitor: records readiness delay, window time and WHY shooting was blocked", s -> {
            Storage.setTarget(0, 60, 40);
            s.follower.pose = new Pose(10, 10, 0);
            SimCalContext c = new SimCalContext(s);
            ProcedureResult r = ShootWindowProcedures.monitor(8, 0).run(c);
            check(!r.refused && !r.aborted, "ran " + r.notes);
            info("delay " + r.metrics.get("readiness_delay_s") + " s, in window " + r.metrics.get("time_in_window_s") + " s, notes " + r.notes);
            check(r.metrics.get("readiness_delay_s") > 0.5 && r.metrics.get("readiness_delay_s") < 4, "flywheel spin-up delay measured");
            check(r.metrics.get("time_in_window_s") > 3, "time in window");
            check(r.metrics.get("window_entries") == 1, "one entry");
            check(String.join(" ", r.notes).contains("flywheel not at stable velocity"), "blocking reason recorded");
            check(s.shotsFired == 0 && s.intake.power == 0 && s.stopper.position == RobotConfig.STOPPER_CLOSED, "observation only: never fed");
            check(!String.join(" ", r.notes).contains(": ready"), "'ready' is never reported as a blocking reason");
            near(0, Storage.flywheelTargetVelocity, 0, "flywheel stopped afterwards");
        });

        sim("cal window monitor: zone entry/exit is measured and the outside reason is reported", s -> {
            zone();
            Storage.setTarget(0, 60, 40);
            s.follower.pose = new Pose(100, 100, 0);
            SimCalContext c = new SimCalContext(s);
            c.onLoop = x -> { if (x.nanos > 5_000_000_000L + 1_000_000_000L && x.nanos < 7_500_000_000L) x.follower.pose = new Pose(20, 20, 0); else x.follower.pose = new Pose(100, 100, 0); };
            ProcedureResult r = ShootWindowProcedures.monitor(9, 0).run(c);
            check(r.metrics.get("window_entries") == 1, "entered once: " + r.metrics);
            check(r.metrics.get("time_in_window_s") > 1.0 && r.metrics.get("time_in_window_s") < 3.0, "time inside ~2.5 s: " + r.metrics.get("time_in_window_s"));
            check(r.metrics.get("time_outside_window_s") > 5, "time outside");
            check(String.join(" ", r.notes).contains("outside shooting zone"), "reason: " + r.notes);
        });

        sim("cal window shoot: shot outcomes counted; a missing ball shows as a lower success rate with the failure reason", s -> {
            Storage.setTarget(0, 60, 40);
            s.follower.pose = new Pose(10, 10, 0);
            s.ballsLoaded = 1;   // only one of the two balls is really there
            SimCalContext c = new SimCalContext(s);
            ProcedureResult r = ShootWindowProcedures.monitor(25, 2).run(c);
            near(1, r.metrics.get("shots_ok"), 0, "one shot made");
            near(0.5, r.metrics.get("shot_success_rate"), 1e-9, "50%");
            check(String.join(" ", r.notes).contains("failure: ShootBalls: no RPM drop"), "failure reason: " + r.notes);
            check(s.intake.power == 0 && s.stopper.position == RobotConfig.STOPPER_CLOSED, "mechanism safe afterwards");
            ProcedureResult bad = ShootWindowProcedures.monitor(25, 9).run(new SimCalContext(s));
            check(bad.refused, "ball count above the limit refused");
        });

        sim("cal moving shot stage 1/2: stationary flywheel+turret, then stationary shooting", s -> {
            Storage.setTarget(0, 60, 40);
            s.follower.pose = new Pose(10, 10, 0);
            s.ballsLoaded = 3;
            ProcedureResult r1 = MovingShotProcedures.stage(1, null, null, 1, 3000, 10).run(new SimCalContext(s));
            check(!r1.refused && r1.metrics.get("completed_normally") == 1.0, "stage 1 completes: " + r1.notes);
            check(s.shotsFired == 0, "stage 1 never feeds");
            ProcedureResult r2 = MovingShotProcedures.stage(2, null, null, 3, 3000, 25).run(new SimCalContext(s));
            near(3, r2.metrics.get("shots_ok"), 0, "three balls");
            near(1, r2.metrics.get("shot_success_rate"), 1e-9, "100%");
            check(s.stopper.position == RobotConfig.STOPPER_CLOSED && s.intake.power == 0, "safe");
        });

        sim("cal moving shot stages 3-6: path + spin-up, moving shot, multi-ball, full cycle (production commands, stub follower)", s -> {
            Storage.setTarget(0, 80, 40);
            final Pose start = new Pose(10, 10, 0), end = new Pose(50, 10, 0);
            s.follower.followDurationLoops = 300;   // each path takes 3 s
            s.ballsLoaded = 10;
            SimCalContext base = new SimCalContext(s);
            base.onLoop = x -> { x.follower.vel = new Velocity(13, 0, 0); x.follower.pose = new Pose(10 + Math.min(40, x.follower.loopsSinceFollow * 0.13), 10, 0); };
            ProcedureResult r3 = MovingShotProcedures.stage(3, start, end, 1, 3000, 15).run(base);
            check(!r3.refused && r3.metrics.get("path_completed") == 1.0 && s.shotsFired == 0, "stage 3: path done, no feeding: " + r3.notes);
            check(r3.metrics.get("max_speed_in_s") >= 12, "speed recorded");

            ProcedureResult r4 = MovingShotProcedures.stage(4, start, end, 1, 3000, 20).run(base);
            near(1, r4.metrics.get("shots_ok"), 0, "stage 4 one shot while driving");
            check(r4.metrics.get("completed_normally") == 1.0, "stage 4 completes: " + r4.notes);

            ProcedureResult r5 = MovingShotProcedures.stage(5, start, end, 3, 3000, 30).run(base);
            near(3, r5.metrics.get("shots_ok"), 0, "stage 5 three shots while driving: " + r5.notes);
            check(r5.metrics.get("first_shot_after_s") < r5.metrics.get("run_time_s"), "timeline recorded");

            int paths = s.follower.followCalls;
            ProcedureResult r6 = MovingShotProcedures.stage(6, start, end, 2, 3000, 40).run(base);
            check(r6.metrics.get("completed_normally") == 1.0, "stage 6 completes: " + r6.notes);
            check(s.follower.followCalls - paths == 2, "cycle drove two paths");
            near(0, s.intake.power, 0, "intake stopped when the pickup path ended");
            check(r6.metrics.get("shots_ok") == 2, "cycle shot its balls: " + r6.metrics);
        });

        sim("cal moving shot: preflight refusals and a mid-stage abort leave everything safe", s -> {
            Pose a = new Pose(0, 0, 0), b = new Pose(40, 0, 0);
            check(MovingShotProcedures.stage(4, a, b, 1, 3000, 10).run(new SimCalContext(s)).refused, "no target point set");
            Storage.setTarget(0, 80, 40);
            check(MovingShotProcedures.stage(4, a, new Pose(1, 0, 0), 1, 3000, 10).run(new SimCalContext(s)).refused, "path too short");
            check(MovingShotProcedures.stage(4, null, b, 1, 3000, 10).run(new SimCalContext(s)).refused, "missing pose");
            check(MovingShotProcedures.stage(9, a, b, 1, 3000, 10).run(new SimCalContext(s)).refused, "bad stage");
            check(MovingShotProcedures.stage(2, a, b, 9, 3000, 10).run(new SimCalContext(s)).refused, "bad ball count");
            check(s.follower.followCalls == 0 && s.stopper.writes == 0, "nothing moved");
            s.follower.followDurationLoops = 800;
            s.ballsLoaded = 3;
            SimCalContext ab = new SimCalContext(s);
            ab.abortAtSec = ab.timeSec() + 3.0;
            ProcedureResult r = MovingShotProcedures.stage(5, a, b, 3, 3000, 30).run(ab);
            check(r.aborted, "aborted");
            check(s.follower.holdCalls >= 1, "drivetrain told to hold position");
            near(0, s.intake.power, 0, "feed stopped");
            near(RobotConfig.STOPPER_CLOSED, s.stopper.position, 0, "stopper closed");
            near(0, Storage.flywheelTargetVelocity, 0, "flywheel stopped");
            check(!TurretConfig.HARDWARE_CONFIGURED, "interlock untouched");
        });
    }
}
