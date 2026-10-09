package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.near;
import static org.firstinspires.ftc.teamcode.commands.SimSupport.*;

import org.firstinspires.ftc.teamcode.calibration.Preflight;
import org.firstinspires.ftc.teamcode.control.FlywheelConfig;
import org.firstinspires.ftc.teamcode.control.TurretConfig;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;
import org.firstinspires.ftc.teamcode.tuning.ConfigValidator;
import org.firstinspires.ftc.teamcode.tuning.ParamRegistry;

import java.util.TreeMap;

/** Hardware presence, safe initial state and configuration validity. (RobotHardware.init itself needs a real HardwareMap: robot-only.) */
public final class HardwareConfigTests {
    private HardwareConfigTests() {}

    public static void run() {
        sim("hardware: the committed defaults are a valid configuration and every controller designs from them", s -> {
            check(ConfigValidator.validate(new TreeMap<>()).isEmpty(), "defaults validate");
            check(ConfigValidator.controllersBuild(new TreeMap<>()).isEmpty(), "turret and flywheel controllers build from the placeholder defaults");
        });

        sim("hardware: unvalidated paths ship disabled (interlocks off, gates off, targets unset)", s -> {
            check(!TurretConfig.HARDWARE_CONFIGURED, "turret never powered by default");
            check(!FlywheelConfig.MODEL_IDENTIFIED, "flywheel stays on the SDK PIDF path by default");
            check(!RobotConfig.REQUIRE_TURRET_ALIGNED && !RobotConfig.SHOOT_ZONE_ENABLED && !RobotConfig.REQUIRE_LOCALIZATION, "readiness gates off");
            check(!Storage.targetA.isValid() && !Storage.targetB.isValid(), "no invented target coordinates");
            check(RobotConfig.STOPPER_OPEN != RobotConfig.STOPPER_CLOSED, "distinct stopper positions");
            check(!RobotConfig.DETECT_INTAKE_JAM, "jam detection off until a threshold is measured");
        });

        sim("hardware: preflight names exactly the missing device for every subsystem", s -> {
            RobotHardware r = s.robot;
            check(Preflight.check(r, Preflight.Need.TURRET, Preflight.Need.FLYWHEEL, Preflight.Need.FEED, Preflight.Need.STOPPER,
                    Preflight.Need.LOCALIZATION, Preflight.Need.BATTERY).isEmpty(), "complete robot passes");
            r.turret = null;
            check(Preflight.check(r, Preflight.Need.TURRET).get(0).contains("turret"), "turret");
            r.shooterRight = null;
            check(Preflight.check(r, Preflight.Need.FLYWHEEL).get(0).contains("shooter"), "shooters");
            r.intakeAndTransferMotor = null;
            check(Preflight.check(r, Preflight.Need.FEED).get(0).contains("intake"), "intake");
            r.stopper = null;
            check(Preflight.check(r, Preflight.Need.STOPPER).get(0).contains("stopper"), "stopper");
            r.follower = null;
            check(Preflight.check(r, Preflight.Need.LOCALIZATION).get(0).contains("follower"), "localization");
            r.batteryVoltage = null;
            check(Preflight.check(r, Preflight.Need.BATTERY).get(0).contains("unreadable"), "battery sensor");
            check(Preflight.check(null, Preflight.Need.TURRET).get(0).contains("null"), "null robot");
        });

        sim("hardware: preflight checks operating conditions (battery level, shot in progress)", s -> {
            s.vbat = 10.0;
            check(Preflight.check(s.robot, Preflight.Need.BATTERY).get(0).contains("below"), "low battery");
            s.vbat = 12.0;
            Storage.shootInProgress = true;
            check(Preflight.check(s.robot, Preflight.Need.FEED).get(0).contains("shot is in progress"), "shot in progress");
        });

        sim("hardware: placeholder detection follows whether a value was explicitly calibrated", s -> {
            check(Preflight.placeholdersInUse("turret.SHAFT_TICKS_PER_REV", "turret.MIN_ANGLE_RAD").size() == 2, "both placeholders at defaults");
            TurretConfig.SHAFT_TICKS_PER_REV = 537.7;
            check(Preflight.placeholdersInUse("turret.SHAFT_TICKS_PER_REV", "turret.MIN_ANGLE_RAD").size() == 1, "one calibrated");
            check(Preflight.placeholdersInUse("turret.MAX_ANGLE_RAD", "robot.FEED_POWER").size() == 1, "non-placeholder keys never listed");
        });

        sim("hardware: safeShutdown tolerates missing devices and puts present ones in the defined safe state", s -> {
            s.intake.power = 0.8; s.stopper.position = RobotConfig.STOPPER_OPEN; s.turretM.power = 0.4;
            Storage.shootInProgress = true;
            s.robot.shooterLeft = null; s.robot.hood = null;
            s.robot.safeShutdown(true);
            near(0, s.intake.power, 0, "feed off");
            near(0, s.turretM.power, 0, "turret off");
            near(RobotConfig.STOPPER_CLOSED, s.stopper.position, 0, "stopper closed");
            check(!Storage.shootInProgress, "state cleared");
            near(0, s.shooterR.velCmd, 0, "right shooter stopped despite the left one missing");
            new RobotHardware().safeShutdown(true);   // everything null: must not throw
        });

        sim("hardware: invalid configuration values never reach the live parameters", s -> {
            java.util.Map<String, Double> bad = new TreeMap<>();
            bad.put("turret.SHAFT_TICKS_PER_REV", Double.NaN);
            check(!ConfigValidator.validate(bad).isEmpty(), "NaN refused");
            bad.clear(); bad.put("turret.SHAFT_REVS_PER_TURRET_REV", 0.0);
            check(!ConfigValidator.validate(bad).isEmpty(), "zero gear ratio refused (would divide by zero)");
            bad.clear(); bad.put("flywheel.FLYWHEEL_INERTIA_KG_M2", -1.0);
            check(!ConfigValidator.validate(bad).isEmpty(), "negative inertia refused");
            bad.clear(); bad.put("robot.FEED_POWER", 1.5);
            check(!ConfigValidator.validate(bad).isEmpty(), "power above 1 refused");
            near(ParamRegistry.defaultOf("robot.FEED_POWER"), RobotConfig.FEED_POWER, 0, "live value unchanged by validation");
        });
    }
}
