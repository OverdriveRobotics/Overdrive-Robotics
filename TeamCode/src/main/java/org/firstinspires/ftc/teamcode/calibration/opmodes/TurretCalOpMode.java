package org.firstinspires.ftc.teamcode.calibration.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.calibration.*;
import org.firstinspires.ftc.teamcode.commands.RobotConfig;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Turret calibration (HARDWARE). See calibration/turret/README.md. */
@TeleOp(name = "CAL Turret", group = "Calibration")
public class TurretCalOpMode extends CalibrationOpMode {
    @Override protected String title() { return "TURRET CALIBRATION"; }
    @Override protected String[] tunableKeys() {
        return new String[]{"turret.SHAFT_TICKS_PER_REV", "turret.SHAFT_REVS_PER_TURRET_REV", "turret.ENCODER_SIGN", "turret.START_ANGLE_RAD",
                "turret.SHAFT_FREE_SPEED_RAD_S", "turret.SHAFT_STALL_TORQUE_NM", "turret.STALL_CURRENT_A", "turret.LOAD_INERTIA_KG_M2",
                "turret.VISCOUS_FRICTION", "turret.STATIC_FRICTION_VOLTS", "turret.MAX_ANGLE_ERROR_RAD", "turret.MAX_VELOCITY_ERROR_RAD_S",
                "turret.MAX_REFERENCE_VELOCITY_RAD_S", "turret.VELOCITY_FILTER_ALPHA", "turret.INTEGRAL_GAIN",
                "turret.ALIGN_TOLERANCE_RAD", "turret.ALIGN_VELOCITY_TOL_RAD_S", "target.A_X", "target.A_Y", "target.B_X", "target.B_Y",
                "turret.HARDWARE_CONFIGURED", "cal.TURRET_STEP_DEG", "cal.TURRET_KNOWN_ANGLE_DEG", "cal.TURRET_AIM_SECONDS"};
    }
    @Override protected List<Item> items() {
        List<Item> l = new ArrayList<>();
        l.add(new Item("1 Encoder direction", TurretProcedures::encoderDirection));
        l.add(new Item("2 Ticks per turret revolution (by hand)", () -> TurretProcedures.ticksPerRev(3, CalSettings.TURRET_KNOWN_ANGLE_DEG)));
        l.add(new Item("3 Mechanical limits (by hand)", TurretProcedures::limits));
        l.add(new Item("4 Zero / forward-mark check", TurretProcedures::zeroCheck));
        l.add(new Item("5 Pulse system identification", () -> TurretProcedures.sysId(1.5)));
        l.add(new Item("6 Step response (production controller)", () -> TurretProcedures.stepResponse(CalSettings.TURRET_STEP_DEG, 2.5)));
        l.add(new Item("7 Aim accuracy + heading compensation (production AimTurret)", () -> TurretProcedures.aimAccuracy(CalSettings.TURRET_AIM_SECONDS, CalSettings.TURRET_AIM_SECONDS / 2)));
        return l;
    }
}
