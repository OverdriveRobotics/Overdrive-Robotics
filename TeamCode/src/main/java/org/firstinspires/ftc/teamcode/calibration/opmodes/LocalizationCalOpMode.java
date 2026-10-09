package org.firstinspires.ftc.teamcode.calibration.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.calibration.*;
import org.firstinspires.ftc.teamcode.commands.RobotConfig;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Localization / coordinate-frame calibration (HARDWARE). The drivetrain is never powered: push the robot by hand. */
@TeleOp(name = "CAL Localization", group = "Calibration")
public class LocalizationCalOpMode extends CalibrationOpMode {
    @Override protected String title() { return "LOCALIZATION CALIBRATION"; }
    @Override protected String[] tunableKeys() {
        return new String[]{"turret.ROBOT_VELOCITY_IS_FIELD_FRAME", "target.A_X", "target.A_Y", "target.B_X", "target.B_Y"};
    }
    @Override protected List<Item> items() {
        List<Item> l = new ArrayList<>();
        l.add(new Item("1 Velocity coordinate frame (push by hand)", () -> LocalizationProcedures.velocityFrame(15)));
        l.add(new Item("2 Heading: one full revolution", LocalizationProcedures::headingRevolution));
        l.add(new Item("3 Straight-line distance vs tape", LocalizationProcedures::straightDistance));
        l.add(new Item("4 Reference poses (you enter 3 taped poses)", () -> new Procedure() {
            @Override public String name() { return "loc-reference"; }
            @Override public String[] paramKeys() { return new String[0]; }
            @Override public ProcedureResult run(CalContext c) {
                double[][] ref = new double[3][3];
                for (int i = 0; i < 3; i++) {
                    ref[i][0] = c.promptValue("Reference " + (i + 1) + " X (in)", 0, 1, -400, 400);
                    ref[i][1] = c.promptValue("Reference " + (i + 1) + " Y (in)", 0, 1, -400, 400);
                    ref[i][2] = c.promptValue("Reference " + (i + 1) + " heading (deg)", 0, 5, -360, 360);
                }
                return LocalizationProcedures.referencePoses(ref).run(c);
            }
        }));
        l.add(new Item("5 Measure target points A and B", LocalizationProcedures::measureTargets));
        return l;
    }
}
