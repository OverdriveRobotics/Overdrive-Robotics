package org.firstinspires.ftc.teamcode.control;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.test;

/** Pure shooting-window logic: every condition, its precedence, boundaries and NaN handling. */
public final class ShootingReadinessTests {
    private ShootingReadinessTests() {}

    private static ShootingReadiness.Inputs ready() {
        ShootingReadiness.Inputs in = new ShootingReadiness.Inputs();
        in.requireLocalization = in.requireZone = in.requireAim = true;
        in.poseFinite = true; in.poseAgeMs = 10;
        in.x = 25; in.y = 25; in.minX = 0; in.maxX = 50; in.minY = 0; in.maxY = 50;
        in.speed = 5; in.maxSpeed = 30;
        in.targetValid = true; in.turretSensorValid = true; in.turretReachable = true;
        in.turretErrorRad = 0.005; in.turretVelRadS = 0.1;
        in.flywheelTarget = 1800; in.flywheelStableFresh = true;
        return in;
    }

    private static String why(ShootingReadiness.Inputs in) { return ShootingReadiness.evaluate(in).reason; }

    public static void run() {
        test("readiness: all conditions satisfied -> ready", () -> check(ShootingReadiness.evaluate(ready()).ready, "ready"));

        test("readiness: each single failing condition blocks the shot and names itself", () -> {
            ShootingReadiness.Inputs in;
            in = ready(); in.poseFinite = false; check(why(in).contains("localization invalid"), "pose invalid");
            in = ready(); in.poseAgeMs = 500; check(why(in).contains("stale"), "pose stale");
            in = ready(); in.x = 60; check(why(in).contains("outside shooting zone"), "outside x");
            in = ready(); in.y = -1; check(why(in).contains("outside shooting zone"), "outside y");
            in = ready(); in.speed = 31; check(why(in).contains("too fast"), "too fast");
            in = ready(); in.speed = Double.NaN; check(why(in).contains("too fast"), "NaN speed never passes");
            in = ready(); in.targetValid = false; check(why(in).contains("target invalid"), "target");
            in = ready(); in.turretSensorValid = false; check(why(in).contains("sensor"), "turret sensor");
            in = ready(); in.turretReachable = false; check(why(in).contains("range"), "unreachable");
            in = ready(); in.turretErrorRad = 0.1; check(why(in).contains("not aligned"), "aim error");
            in = ready(); in.turretVelRadS = 3; check(why(in).contains("not aligned"), "turret still moving");
            in = ready(); in.turretFault = "boom"; check(why(in).contains("turret fault"), "turret fault");
            in = ready(); in.flywheelTarget = 0; check(why(in).contains("target not set"), "no flywheel target");
            in = ready(); in.flywheelStableFresh = false; check(why(in).contains("stable"), "flywheel not stable");
            in = ready(); in.flywheelFault = "dead"; check(why(in).contains("flywheel fault"), "flywheel fault outranks everything");
            in = ready(); in.feedPermits = false; in.feedBlockReason = "shot in progress"; check(why(in).contains("shot in progress"), "feed busy");
        });

        test("readiness: zone edges are inclusive, one step outside is not", () -> {
            ShootingReadiness.Inputs in = ready();
            in.x = 0; in.y = 50; check(ShootingReadiness.evaluate(in).ready, "corner inclusive");
            in.x = -0.001; check(!ShootingReadiness.evaluate(in).ready, "just outside");
            in = ready(); in.x = Double.NaN; check(!ShootingReadiness.evaluate(in).ready, "NaN position is outside");
        });

        test("readiness: gates that are switched off do not block (pre-turret behaviour preserved)", () -> {
            ShootingReadiness.Inputs in = ready();
            in.requireLocalization = in.requireZone = in.requireAim = false;
            in.poseFinite = false; in.x = 1e9; in.targetValid = false; in.turretSensorValid = false; in.turretFault = "ignored";
            check(ShootingReadiness.evaluate(in).ready, "only the flywheel matters when the gates are off");
            in.flywheelStableFresh = false;
            check(!ShootingReadiness.evaluate(in).ready, "flywheel always required");
        });

        test("readiness: heading/speed constraint applies only with the zone or aim gate", () -> {
            ShootingReadiness.Inputs in = ready();
            in.requireZone = false; in.requireAim = false; in.requireLocalization = true; in.speed = 100;
            check(ShootingReadiness.evaluate(in).ready, "speed ignored when neither zone nor aim is required");
        });
    }
}
