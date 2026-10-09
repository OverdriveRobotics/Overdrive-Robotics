package org.firstinspires.ftc.teamcode.calibration.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.calibration.*;
import org.firstinspires.ftc.teamcode.commands.RobotConfig;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Intake / transfer calibration (HARDWARE). */
@TeleOp(name = "CAL Intake", group = "Calibration")
public class IntakeCalOpMode extends CalibrationOpMode {
    @Override protected String title() { return "INTAKE / TRANSFER CALIBRATION"; }
    @Override protected String[] tunableKeys() {
        return new String[]{"robot.INTAKE_POWER", "robot.FEED_POWER", "robot.DETECT_INTAKE_JAM", "robot.INTAKE_JAM_AMPS", "robot.INTAKE_JAM_MS"};
    }
    @Override protected List<Item> items() {
        List<Item> l = new ArrayList<>();
        l.add(new Item("1 Direction + 4 transfer cycles (1.5 s each)", () -> IntakeProcedures.directionAndTransfer(4, 1500)));
        l.add(new Item("2 Direction + 8 short transfers (0.8 s)", () -> IntakeProcedures.directionAndTransfer(8, 800)));
        return l;
    }
}
