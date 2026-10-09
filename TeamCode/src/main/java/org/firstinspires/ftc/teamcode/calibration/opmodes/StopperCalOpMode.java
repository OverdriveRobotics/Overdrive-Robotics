package org.firstinspires.ftc.teamcode.calibration.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.calibration.*;
import org.firstinspires.ftc.teamcode.commands.RobotConfig;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Stopper servo calibration (HARDWARE). Timings are operator-observed: there is no servo feedback. */
@TeleOp(name = "CAL Stopper", group = "Calibration")
public class StopperCalOpMode extends CalibrationOpMode {
    @Override protected String title() { return "STOPPER CALIBRATION"; }
    @Override protected String[] tunableKeys() {
        return new String[]{"robot.STOPPER_CLOSED", "robot.STOPPER_OPEN", "robot.STOPPER_SETTLE_MS", "robot.STOPPER_CLOSE_SETTLE_MS"};
    }
    @Override protected List<Item> items() {
        List<Item> l = new ArrayList<>();
        l.add(new Item("1 Jog + set open/closed positions (slow)", StopperProcedures::endpoints));
        l.add(new Item("2 Operator-timed open/close delays (5 repeats)", () -> StopperProcedures.timing(5)));
        return l;
    }
}
