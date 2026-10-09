package org.firstinspires.ftc.teamcode.calibration;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;
import org.firstinspires.ftc.teamcode.tuning.ParamRegistry;

import java.util.ArrayList;
import java.util.List;

/** Verifies required hardware and operating conditions BEFORE any procedure moves anything. */
public final class Preflight {
    private Preflight() {}

    public enum Need { TURRET, FLYWHEEL, FEED, STOPPER, LOCALIZATION, BATTERY }

    /** Returns blocking problems (empty = go). */
    public static List<String> check(RobotHardware r, Need... needs) {
        List<String> p = new ArrayList<>();
        if (r == null) { p.add("robot hardware object is null"); return p; }
        for (Need n : needs) {
            switch (n) {
                case TURRET: if (r.turret == null) p.add("turret motor not found"); break;
                case FLYWHEEL: if (r.shooterLeft == null || r.shooterRight == null) p.add("shooter motors not found"); break;
                case FEED: if (r.intakeAndTransferMotor == null) p.add("intake/transfer motor not found"); break;
                case STOPPER: if (r.stopper == null) p.add("stopper servo not found"); break;
                case LOCALIZATION: if (r.follower == null) p.add("Pedro follower not available"); break;
                case BATTERY: {
                    double v = r.batteryVolts();
                    if (!Double.isFinite(v)) p.add("battery voltage unreadable");
                    else if (v < CalConfig.MIN_BATTERY_V) p.add(String.format("battery %.2f V is below the %.1f V minimum", v, CalConfig.MIN_BATTERY_V));
                    break;
                }
            }
        }
        if (Storage.shootInProgress) p.add("a shot is in progress");
        return p;
    }

    /** Keys among {@code keys} whose live value still equals the compiled (placeholder) default. */
    public static List<String> placeholdersInUse(String... keys) {
        List<String> l = new ArrayList<>();
        for (String k : keys) {
            org.firstinspires.ftc.teamcode.tuning.Param p = ParamRegistry.get(k);
            if (p != null && p.placeholder && (p.get() == ParamRegistry.defaultOf(k)
                    || (Double.isNaN(p.get()) && Double.isNaN(ParamRegistry.defaultOf(k))))) l.add(k);
        }
        return l;
    }
}
