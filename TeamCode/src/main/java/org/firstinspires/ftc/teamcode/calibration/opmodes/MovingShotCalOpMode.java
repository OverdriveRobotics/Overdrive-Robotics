package org.firstinspires.ftc.teamcode.calibration.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.calibration.*;
import org.firstinspires.ftc.teamcode.commands.RobotConfig;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Staged moving-shot validation (HARDWARE; stages 3-6 DRIVE the robot). Set cal.PATH_* first (field measurements). */
@TeleOp(name = "CAL Moving Shot", group = "Calibration")
public class MovingShotCalOpMode extends CalibrationOpMode {
    @Override protected String title() { return "MOVING-SHOT STAGED VALIDATION"; }
    @Override protected String[] tunableKeys() {
        return new String[]{"cal.PATH_START_X", "cal.PATH_START_Y", "cal.PATH_START_HEADING_DEG", "cal.PATH_END_X", "cal.PATH_END_Y",
                "cal.PATH_END_HEADING_DEG", "cal.STAGE_BALLS", "cal.WINDOW_MS", "cal.STAGE_SECONDS", "target.A_X", "target.A_Y",
                "robot.FLYWHEEL_VELOCITY_A", "robot.MAX_SHOOT_SPEED", "robot.REQUIRE_TURRET_ALIGNED", "robot.SHOOT_ZONE_ENABLED",
                "robot.READY_TIMEOUT_MS", "turret.ALIGN_TOLERANCE_RAD"};
    }
    @Override protected List<Item> items() {
        List<Item> l = new ArrayList<>();
        for (int s = 1; s <= 6; s++) {
            final int stage = s;
            l.add(new Item("Stage " + MovingShotProcedures.STAGE_NAMES[s], () -> MovingShotProcedures.stage(stage, CalSettings.start(), CalSettings.end(),
                    CalSettings.STAGE_BALLS, CalSettings.WINDOW_MS, CalSettings.STAGE_SECONDS)));
        }
        return l;
    }
}
