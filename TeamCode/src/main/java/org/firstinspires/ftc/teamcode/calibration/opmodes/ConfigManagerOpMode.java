package org.firstinspires.ftc.teamcode.calibration.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.calibration.*;
import org.firstinspires.ftc.teamcode.commands.RobotConfig;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Configuration manager (HARDWARE): inspect/accept/reject/restore without running any procedure. */
@TeleOp(name = "CAL Config Manager", group = "Calibration")
public class ConfigManagerOpMode extends CalibrationOpMode {
    @Override protected String title() { return "CONFIGURATION MANAGER"; }
    @Override protected String[] tunableKeys() { return new String[0]; }
    @Override protected List<Item> items() { return new ArrayList<>(); }
}
