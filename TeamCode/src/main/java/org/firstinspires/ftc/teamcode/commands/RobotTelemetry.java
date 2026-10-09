package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

import java.util.Locale;

/** Compact shooter telemetry from Storage (no hardware reads, so it does not add bulk-read traffic). */
public final class RobotTelemetry {
    private RobotTelemetry() {}

    private static String f(double v) { return Double.isFinite(v) ? String.format(Locale.US, "%.1f", v) : "n/a"; }
    private static String deg(double rad) { return Double.isFinite(rad) ? f(Math.toDegrees(rad)) : "n/a"; }

    public static void add(Telemetry t, RobotHardware robot) {
        Storage.FieldPoint tp = Storage.activeTarget();
        Pose p = robot != null && robot.follower != null ? robot.follower.pose() : null;
        t.addData("target", (Storage.activeTargetIndex == 0 ? "A " : "B ") + tp);
        if (p != null) t.addData("pose", f(p.x()) + ", " + f(p.y()) + " @ " + deg(p.heading()) + " deg");
        t.addData("turret deg des/meas/err", deg(Storage.turretDesiredAngle) + " / " + deg(Storage.turretAngle)
                + " / " + deg(Storage.turretError));
        t.addData("turret vel/pwr/sat", deg(Storage.turretVelocity) + " dps / " + f(Storage.turretPower)
                + (Storage.turretSaturated ? " SAT" : ""));
        if (Storage.turretFault != null) t.addData("turret fault", Storage.turretFault);
        t.addData("flywheel tgt/meas/pwr", f(Storage.flywheelTargetVelocity) + " / " + f(Storage.flywheelMeasuredVelocity)
                + " / " + f(Storage.flywheelPower) + " [" + Storage.flywheelMode + "]");
        t.addData("flywheel stable ms", f(Storage.flywheelStableForMs) + (Storage.flywheelReady ? " READY" : ""));
        if (Storage.flywheelFault != null) t.addData("flywheel fault", Storage.flywheelFault);
        t.addData("shoot", Storage.shootPhase + "  shots " + Storage.ballsShot + " (total " + Storage.totalBallsShot + ")");
        t.addData("ready to shoot", Storage.readyToShoot ? "YES" : "no: " + Storage.notReadyReason);
        if (Storage.lastFailure != null) t.addData("last failure", Storage.lastFailure);
    }
}
