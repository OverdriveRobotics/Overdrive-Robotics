package org.firstinspires.ftc.teamcode;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Entry point: scripts/run_tests.sh [suite ...]. With no arguments every suite runs.
 * Suites are registered in SUITES; add a line there for a new suite.
 */
public final class AllTests {
    public static final Map<String, Runnable> SUITES = new LinkedHashMap<>();
    static {
        SUITES.put("config", org.firstinspires.ftc.teamcode.tuning.ConfigFrameworkTests::run);
        SUITES.put("metrics", org.firstinspires.ftc.teamcode.tuning.MetricsSysIdTests::run);
        SUITES.put("control", org.firstinspires.ftc.teamcode.control.ControlTests::run);
        SUITES.put("readiness", org.firstinspires.ftc.teamcode.control.ShootingReadinessTests::run);
        SUITES.put("turret-math", org.firstinspires.ftc.teamcode.control.TurretGeometryAndControllerTests::run);
        SUITES.put("flywheel-observer", org.firstinspires.ftc.teamcode.control.FlywheelObserverTests::run);
        SUITES.put("flywheel", org.firstinspires.ftc.teamcode.commands.FlywheelRegulatorTests::run);
        SUITES.put("shooting", org.firstinspires.ftc.teamcode.commands.ShootingStateMachineTests::run);
        SUITES.put("validation", org.firstinspires.ftc.teamcode.commands.ValidationTests::run);
        SUITES.put("intake", org.firstinspires.ftc.teamcode.commands.IntakeTests::run);
        SUITES.put("turret", org.firstinspires.ftc.teamcode.commands.TurretAimingTests::run);
        SUITES.put("movingshot", org.firstinspires.ftc.teamcode.commands.MovingShotTests::run);
        SUITES.put("cal-flywheel", org.firstinspires.ftc.teamcode.commands.FlywheelCalibrationTests::run);
        SUITES.put("cal-turret", org.firstinspires.ftc.teamcode.commands.TurretCalibrationTests::run);
        SUITES.put("cal-intake-stopper", org.firstinspires.ftc.teamcode.commands.IntakeStopperCalibrationTests::run);
        SUITES.put("cal-localization", org.firstinspires.ftc.teamcode.commands.LocalizationCalibrationTests::run);
        SUITES.put("cal-window-movingshot", org.firstinspires.ftc.teamcode.commands.WindowMovingShotCalibrationTests::run);
        SUITES.put("workflow", org.firstinspires.ftc.teamcode.commands.CalibrationWorkflowTests::run);
        SUITES.put("hardware", org.firstinspires.ftc.teamcode.commands.HardwareConfigTests::run);
        SUITES.put("ivy", org.firstinspires.ftc.teamcode.commands.IvyTests::run);
        SUITES.put("state", org.firstinspires.ftc.teamcode.commands.StateSafetyTests::run);
        SUITES.put("opmode-exit", org.firstinspires.ftc.teamcode.commands.OpModeLifecycleTests::run);
        SUITES.put("auto-routines", org.firstinspires.ftc.teamcode.commands.AutoRoutineTests::run);
        SUITES.put("shoot-cancel", org.firstinspires.ftc.teamcode.commands.ShootingCancelTests::run);
        SUITES.put("lifecycle", org.firstinspires.ftc.teamcode.commands.LifecycleTests::run);
    }

    public static void main(String[] args) {
        if (args.length == 1 && args[0].equals("--list")) { for (String k : SUITES.keySet()) System.out.println(k); return; }
        java.util.List<String> want = args.length == 0 ? new java.util.ArrayList<>(SUITES.keySet()) : java.util.Arrays.asList(args);
        for (String s : want) {
            Runnable r = SUITES.get(s);
            if (r == null) { System.out.println("unknown suite '" + s + "'. Known: " + SUITES.keySet()); System.exit(2); }
            System.out.println("=== suite: " + s);
            r.run();
        }
        System.out.println("passed " + T.passed + ", failed " + T.failed);
        for (String f : T.failures) System.out.println("  FAILED: " + f);
        System.exit(T.failed == 0 ? 0 : 1);
    }
}
