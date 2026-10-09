package org.firstinspires.ftc.teamcode.calibration.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.calibration.*;
import org.firstinspires.ftc.teamcode.commands.RobotConfig;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Flywheel calibration (HARDWARE). See calibration/flywheel/README.md. */
@TeleOp(name = "CAL Flywheel", group = "Calibration")
public class FlywheelCalOpMode extends CalibrationOpMode {
    @Override protected String title() { return "FLYWHEEL CALIBRATION"; }
    @Override protected String[] tunableKeys() {
        return new String[]{"robot.FLYWHEEL_VELOCITY_A", "robot.FLYWHEEL_TOLERANCE", "robot.FLYWHEEL_STABLE_MS", "robot.FLYWHEEL_SPINUP_TIMEOUT_MS",
                "flywheel.MAX_VELOCITY_ERROR", "flywheel.MODEL_STD", "flywheel.MEASUREMENT_STD", "flywheel.USE_SYSID_MODEL",
                "flywheel.KS_VOLTS", "flywheel.KV_VOLTS_PER_TICK_S", "flywheel.KA_VOLTS_PER_TICK_S2", "flywheel.SHAFT_TICKS_PER_REV",
                "flywheel.MODEL_IDENTIFIED", "cal.FLYWHEEL_STEP_HOLD_S", "cal.DISTURBANCE_MS"};
    }
    @Override protected List<Item> items() {
        List<Item> l = new ArrayList<>();
        l.add(new Item("1 Encoder scaling check (tachometer)", () -> FlywheelProcedures.scaling(0.3)));
        l.add(new Item("2 Open-loop system identification", () -> FlywheelProcedures.sysId(new double[]{0.35, 0.5, 0.65}, 3, 4)));
        l.add(new Item("3 Step response (production regulator)", () -> FlywheelProcedures.stepResponse(RobotConfig.FLYWHEEL_VELOCITY_A, CalSettings.FLYWHEEL_STEP_HOLD_S)));
        l.add(new Item("4 Disturbance recovery (power cut)", () -> FlywheelProcedures.disturbance(RobotConfig.FLYWHEEL_VELOCITY_A, (int) CalSettings.DISTURBANCE_MS)));
        return l;
    }
}
