package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.info;
import static org.firstinspires.ftc.teamcode.T.near;
import static org.firstinspires.ftc.teamcode.commands.SimSupport.*;

import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.calibration.LocalizationProcedures;
import org.firstinspires.ftc.teamcode.calibration.ProcedureResult;
import org.firstinspires.ftc.teamcode.control.TurretConfig;
import org.firstinspires.ftc.teamcode.control.TurretUtil;

/** Localization procedures (simulated pushes) and the verification of Pedro's velocity frame with the REAL Pedro classes. */
public final class LocalizationCalibrationTests {
    private LocalizationCalibrationTests() {}

    public static void run() {
        sim("pedro frame (software-verified): MotionState.ofVelocity treats the Velocity as FIELD frame and derives the robot-frame Twist", s -> {
            // Field velocity +Y while facing +90 deg: the robot is driving FORWARD, i.e. robot-frame vx = +speed, vy = 0.
            Pose p = new Pose(0, 0, Math.PI / 2);
            MotionState ms = MotionState.ofVelocity(p, new Velocity(0, 10, 0.5));
            near(10, ms.velocity().vy, 1e-9, "velocity() returns the field-frame input unchanged");
            near(10, ms.twist().vx, 1e-9, "twist() is robot-frame forward speed");
            near(0, ms.twist().vy, 1e-9, "no robot-frame sideways speed");
            near(0.5, ms.twist().omega, 1e-12, "turn rate unchanged");
            // Field +X while facing +90 deg: robot moves to its RIGHT -> robot-frame vy = -speed.
            MotionState side = MotionState.ofVelocity(p, new Velocity(10, 0, 0));
            near(-10, side.twist().vy, 1e-9, "sideways motion lands on robot-frame vy");
        });

        sim("pedro frame: TurretUtil.robotToFieldVelocity agrees with Pedro's own Twist.toVelocity at several headings", s -> {
            for (double h : new double[]{0, 0.7, Math.PI / 2, 2.5, -1.9, Math.PI}) {
                Twist tw = new Twist(7, -3, 0.2);
                Velocity v = tw.toVelocity(h);
                double[] mine = TurretUtil.robotToFieldVelocity(7, -3, h);
                near(v.vx, mine[0], 1e-9, "vx @ " + h);
                near(v.vy, mine[1], 1e-9, "vy @ " + h);
            }
            check(TurretConfig.ROBOT_VELOCITY_IS_FIELD_FRAME, "production default matches the verified Pedro behaviour (field frame)");
        });

        sim("pedro frame: the turret rate feed-forward is identical for field-frame velocity and the equivalent robot-frame twist", s -> {
            double h = 1.1, rx = 5, ry = 7, tx = 60, ty = 40;
            Velocity field = new Velocity(12, -4, 0.3);
            Twist robot = field.toTwist(h);
            double[] f2 = TurretUtil.robotToFieldVelocity(robot.vx, robot.vy, h);
            near(TurretUtil.desiredRelativeRate(rx, ry, field.vx, field.vy, 0.3, tx, ty),
                    TurretUtil.desiredRelativeRate(rx, ry, f2[0], f2[1], 0.3, tx, ty), 1e-9, "rate");
        });

        sim("cal localization velocity-frame: detects FIELD and ROBOT velocity from pushes at a non-zero heading, and refuses to guess", s -> {
            for (final boolean fieldFrame : new boolean[]{true, false}) {
                final SimRobot sr = fieldFrame ? s : SimRobot.fresh();
                final SimCalContext c = new SimCalContext(sr);
                final double[] k = {0};
                c.onLoop = x -> {
                    k[0] += 0.01;
                    double h = Math.toRadians(90) + 0.2 * Math.sin(k[0]);
                    double fx = 18 * Math.cos(1.5 * k[0]), fy = 18 * Math.sin(1.5 * k[0]);
                    Pose cur = x.follower.pose;
                    x.follower.pose = new Pose(cur.x() + fx * 0.01, cur.y() + fy * 0.01, h);
                    Velocity fieldV = new Velocity(fx, fy, 0);
                    Twist tw = fieldV.toTwist(h);
                    x.follower.vel = fieldFrame ? fieldV : new Velocity(tw.vx, tw.vy, 0);
                };
                ProcedureResult r = LocalizationProcedures.velocityFrame(10).run(c);
                check(!r.refused, "ran");
                double expect = fieldFrame ? 1.0 : 0.0;
                check(r.suggestions.containsKey("turret.ROBOT_VELOCITY_IS_FIELD_FRAME") && r.suggestions.get("turret.ROBOT_VELOCITY_IS_FIELD_FRAME") == expect,
                        (fieldFrame ? "field" : "robot") + " frame verdict: " + r.notes + " " + r.suggestions);
                if (!fieldFrame) sr.cleanup();
            }
            SimCalContext still = new SimCalContext(s);
            ProcedureResult u = LocalizationProcedures.velocityFrame(5).run(still);
            check(u.suggestions.isEmpty() && String.join(" ", u.notes).contains("undetermined"), "no motion: no verdict");
            check(LocalizationProcedures.velocityFrame(1).run(new SimCalContext(s)).refused, "too short refused");
        });

        sim("cal localization reference poses: position and heading errors against taped marks", s -> {
            SimCalContext c = new SimCalContext(s);
            final double[][] expected = {{0, 0, 0}, {48, 24, 90}, {-24, 60, 180}};
            final int[] i = {0};
            c.operator = new SimCalContext.Operator() {
                @Override public boolean confirm(String p, SimRobot x) {
                    double[] e = expected[Math.min(i[0]++, 2)];
                    x.follower.pose = new Pose(e[0] + 0.5, e[1] - 1.0, Math.toRadians(e[2] + 1.5));   // placement + odometry error
                    return true;
                }
                @Override public double value(String p, double init, SimRobot x) { return init; }
            };
            ProcedureResult r = LocalizationProcedures.referencePoses(expected).run(c);
            near(Math.hypot(0.5, 1.0), r.metrics.get("pos_error_mean_in"), 1e-6, "mean position error");
            near(1.5, r.metrics.get("heading_error_max_deg"), 1e-6, "heading error");
            check(LocalizationProcedures.referencePoses(new double[0][]).run(new SimCalContext(s)).refused, "no references refused");
        });

        sim("cal localization heading revolution: accumulated unwrapped heading vs 360 deg", s -> {
            SimCalContext c = new SimCalContext(s);
            final double[] h = {0};
            c.onLoop = x -> { h[0] += Math.toRadians(365.0) / 600.0; x.follower.pose = new Pose(0, 0, h[0]); };   // reads 1.4% too much
            c.operator = new SimCalContext.Operator() {
                @Override public boolean confirm(String p, SimRobot x) { return p.contains("Back on the mark") ? h[0] >= Math.toRadians(365) : true; }
                @Override public double value(String p, double i, SimRobot x) { return i; }
            };
            ProcedureResult r = LocalizationProcedures.headingRevolution().run(c);
            near(365, r.metrics.get("accumulated_deg"), 1.0, "accumulated");
            near(1.39, r.metrics.get("revolution_error_pct"), 0.3, "percent error");
        });

        sim("cal localization distance: scale error against a tape measure", s -> {
            SimCalContext c = new SimCalContext(s);
            c.operator = new SimCalContext.Operator() {
                @Override public boolean confirm(String p, SimRobot x) { if (p.contains("Push it")) x.follower.pose = new Pose(48.96, 0, 0); return true; }
                @Override public double value(String p, double i, SimRobot x) { return 48; }
            };
            ProcedureResult r = LocalizationProcedures.straightDistance().run(c);
            near(2.0, r.metrics.get("scale_error_pct"), 1e-6, "2% over-read");
        });

        sim("cal localization target points: the pose at the goal becomes the proposed coordinates", s -> {
            SimCalContext c = new SimCalContext(s);
            final int[] i = {0};
            c.operator = new SimCalContext.Operator() {
                @Override public boolean confirm(String p, SimRobot x) { x.follower.pose = i[0]++ == 0 ? new Pose(12.5, 130, 0) : new Pose(131, 128.25, 0); return true; }
                @Override public double value(String p, double init, SimRobot x) { return init; }
            };
            ProcedureResult r = LocalizationProcedures.measureTargets().run(c);
            near(12.5, r.suggestions.get("target.A_X"), 0, "A x"); near(130, r.suggestions.get("target.A_Y"), 0, "A y");
            near(131, r.suggestions.get("target.B_X"), 0, "B x"); near(128.25, r.suggestions.get("target.B_Y"), 0, "B y");
            check(!org.firstinspires.ftc.teamcode.hardware.Storage.targetA.isValid(), "live targets not modified by a measurement");
            info("suggestions " + r.suggestions);
        });
    }
}
