package org.firstinspires.ftc.teamcode.calibration.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.calibration.*;
import org.firstinspires.ftc.teamcode.commands.RobotConfig;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Shooting-window inspector (HARDWARE). Observe-only, or observe + shoot when the window opens. */
@TeleOp(name = "CAL Shooting Window", group = "Calibration")
public class ShootWindowCalOpMode extends CalibrationOpMode {
    @Override protected String title() { return "SHOOTING WINDOW CALIBRATION"; }
    @Override protected String[] tunableKeys() {
        return new String[]{"robot.REQUIRE_LOCALIZATION", "robot.SHOOT_ZONE_ENABLED", "robot.SHOOT_ZONE_MIN_X", "robot.SHOOT_ZONE_MAX_X",
                "robot.SHOOT_ZONE_MIN_Y", "robot.SHOOT_ZONE_MAX_Y", "robot.MAX_SHOOT_SPEED", "turret.ALIGN_TOLERANCE_RAD",
                "turret.ALIGN_VELOCITY_TOL_RAD_S", "turret.MAX_POSE_AGE_MS", "robot.FLYWHEEL_TOLERANCE", "robot.FLYWHEEL_STABLE_MS",
                "robot.READY_TIMEOUT_MS", "robot.SHOT_TIMEOUT_MS", "robot.RECOVER_TIMEOUT_MS", "robot.SHOT_DROP_THRESHOLD",
                "robot.FLYWHEEL_VELOCITY_A", "robot.FLYWHEEL_VELOCITY_B", "robot.REQUIRE_TURRET_ALIGNED", "cal.MONITOR_SECONDS"};
    }
    @Override protected List<Item> items() {
        List<Item> l = new ArrayList<>();
        l.add(new Item("1 Monitor only (push the robot around)", () -> ShootWindowProcedures.monitor(CalSettings.MONITOR_SECONDS, 0)));
        l.add(new Item("2 Monitor + shoot 1 ball", () -> ShootWindowProcedures.monitor(CalSettings.MONITOR_SECONDS, 1)));
        l.add(new Item("3 Monitor + shoot 3 balls", () -> ShootWindowProcedures.monitor(CalSettings.MONITOR_SECONDS + 10, 3)));
        return l;
    }
}
