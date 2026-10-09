package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.info;
import static org.firstinspires.ftc.teamcode.T.near;
import static org.firstinspires.ftc.teamcode.commands.SimSupport.*;

import org.firstinspires.ftc.teamcode.calibration.CalConfig;
import org.firstinspires.ftc.teamcode.calibration.IntakeProcedures;
import org.firstinspires.ftc.teamcode.calibration.ProcedureResult;
import org.firstinspires.ftc.teamcode.calibration.StopperProcedures;
import org.firstinspires.ftc.teamcode.hardware.Storage;

import java.util.ArrayList;
import java.util.List;

/** Intake/transfer and stopper calibration procedures against the simulator. SIMULATION ONLY. */
public final class IntakeStopperCalibrationTests {
    private IntakeStopperCalibrationTests() {}

    public static void run() {
        sim("cal intake: direction, transfer reliability and current statistics; power is capped; motor stopped after", s -> {
            s.intake.currentAmps = 4.2;
            SimCalContext c = new SimCalContext(s);
            final double[] maxAbs = {0};
            c.onLoop = x -> maxAbs[0] = Math.max(maxAbs[0], Math.abs(x.intake.power));
            final int[] cycle = {0};
            c.operator = new SimCalContext.Operator() {
                @Override public boolean confirm(String p, SimRobot x) {
                    if (p.contains("transfer correctly")) return ++cycle[0] != 3;   // third transfer fails
                    return true;
                }
                @Override public double value(String p, double i, SimRobot x) { return i; }
            };
            ProcedureResult r = IntakeProcedures.directionAndTransfer(4, 600).run(c);
            check(!r.refused && !r.aborted, "ran " + r.notes);
            near(1, r.metrics.get("direction_ok"), 0, "direction ok");
            near(0.75, r.metrics.get("transfer_success_rate"), 1e-9, "3 of 4 transfers worked");
            near(4.2, r.metrics.get("current_peak_a"), 1e-9, "peak current");
            near(6.5, r.suggestions.get("robot.INTAKE_JAM_AMPS"), 1e-9, "1.5 x peak, rounded to 0.5 A");
            check(maxAbs[0] <= CalConfig.INTAKE_MAX_POWER + 1e-9, "calibration power cap respected: " + maxAbs[0]);
            near(0, s.intake.power, 0, "stopped");
            check(Storage.intakeState == Storage.IntakeState.IDLE, "state idle");
            check(!RobotConfig.DETECT_INTAKE_JAM, "jam detection untouched");
        });

        sim("cal intake: wrong direction is reported as a wiring fact, not a tunable; abort and refusals are safe", s -> {
            SimCalContext c = new SimCalContext(s);
            c.operator = new SimCalContext.Operator() {
                @Override public boolean confirm(String p, SimRobot x) { return !p.contains("pull balls IN"); }
                @Override public double value(String p, double i, SimRobot x) { return i; }
            };
            ProcedureResult r = IntakeProcedures.directionAndTransfer(1, 400).run(c);
            near(0, r.metrics.get("direction_ok"), 0, "flagged");
            check(String.join(" ", r.notes).contains("RobotHardware"), "points at the source-level fix");
            SimCalContext ab = new SimCalContext(s);
            ab.abortAtSec = ab.timeSec() + 0.2;
            IntakeProcedures.directionAndTransfer(2, 3000).run(ab);
            near(0, s.intake.power, 0, "aborted mid-run: motor stopped by the command's end()");
            Storage.shootInProgress = true;
            check(IntakeProcedures.directionAndTransfer(1, 400).run(new SimCalContext(s)).refused, "refused while a shot is in progress");
            Storage.shootInProgress = false;
            check(IntakeProcedures.directionAndTransfer(0, 400).run(new SimCalContext(s)).refused, "bad cycle count");
            check(IntakeProcedures.directionAndTransfer(1, 50).run(new SimCalContext(s)).refused, "run time too short");
            s.robot.intakeAndTransferMotor = null;
            check(IntakeProcedures.directionAndTransfer(1, 400).run(new SimCalContext(s)).refused, "missing motor");
        });

        sim("cal stopper timing: operator-timed delays with margin, never claims servo feedback, ends closed", s -> {
            SimCalContext c = new SimCalContext(s);
            c.operator = new SimCalContext.Operator() {
                @Override public boolean confirm(String p, SimRobot x) {
                    if (p.contains("OPEN settled")) x.run(140);    // operator reaction + servo travel (simulated)
                    if (p.contains("CLOSE settled")) x.run(180);
                    return true;
                }
                @Override public double value(String p, double i, SimRobot x) { return i; }
            };
            ProcedureResult r = StopperProcedures.timing(4).run(c);
            check(!r.refused && !r.aborted, "ran " + r.notes);
            near(140, r.metrics.get("open_mean_ms"), 25, "open");
            near(180, r.metrics.get("close_mean_ms"), 25, "close");
            double sug = r.suggestions.get("robot.STOPPER_SETTLE_MS");
            check(sug >= r.metrics.get("open_max_ms") * 1.19 && sug % 10 == 0, "20% margin, rounded up to 10 ms: " + sug);
            check(String.join(" ", r.notes).contains("OPERATOR-observed"), "states that these are human observations");
            near(RobotConfig.STOPPER_CLOSED, s.stopper.position, 0, "finished closed");
            check(RobotConfig.STOPPER_SETTLE_MS == 150, "live config untouched");
        });

        sim("cal stopper timing: refuses identical positions, abort leaves the stopper closed", s -> {
            RobotConfig.STOPPER_OPEN = RobotConfig.STOPPER_CLOSED;
            check(StopperProcedures.timing(3).run(new SimCalContext(s)).refused, "open == closed refused");
            RobotConfig.STOPPER_OPEN = 0.5;
            SimCalContext ab = new SimCalContext(s);
            ab.operator = new SimCalContext.Operator() {
                @Override public boolean confirm(String p, SimRobot x) { if (p.contains("OPEN settled")) { x.run(50); return false; } return true; }
                @Override public double value(String p, double i, SimRobot x) { return i; }
            };
            ProcedureResult r = StopperProcedures.timing(3).run(ab);
            check(r.aborted, "aborted by the operator");
            near(RobotConfig.STOPPER_CLOSED, s.stopper.position, 0, "closed after abort");
            s.robot.stopper = null;
            check(StopperProcedures.timing(3).run(new SimCalContext(s)).refused, "missing servo");
        });

        sim("cal stopper endpoints: jogging is slew-limited (no jumps), result validates, ends closed", s -> {
            s.stopper.position = 0.0;
            final List<Double> positions = new ArrayList<>();
            SimCalContext c = new SimCalContext(s);
            c.onLoop = x -> positions.add(x.stopper.position);
            c.operator = new SimCalContext.Operator() {
                int asked;
                @Override public boolean confirm(String p, SimRobot x) {
                    if (p.contains("CLOSED position")) return true;                 // keep closed as is
                    if (p.contains("OPEN position")) return ++asked > 1;           // first look: adjust, then keep
                    return true;
                }
                @Override public double value(String p, double i, SimRobot x) { return 0.62; }
            };
            ProcedureResult r = StopperProcedures.endpoints().run(c);
            check(!r.aborted && !r.refused, "ran " + r.notes);
            near(0.62, r.suggestions.get("robot.STOPPER_OPEN"), 1e-9, "open adopted");
            double maxJump = 0;
            for (int i = 1; i < positions.size(); i++) if (!Double.isNaN(positions.get(i)) && !Double.isNaN(positions.get(i - 1)))
                maxJump = Math.max(maxJump, Math.abs(positions.get(i) - positions.get(i - 1)));
            check(maxJump <= CalConfig.STOPPER_JOG_RATE * 0.011 + 1e-9, "never jumped: max step " + maxJump);
            check(org.firstinspires.ftc.teamcode.tuning.ConfigValidator.validate(new java.util.TreeMap<>(r.suggestions)).isEmpty(), "validates");
            near(RobotConfig.STOPPER_CLOSED, s.stopper.position, 0, "closed at the end");
            info("max servo step per loop " + maxJump);
        });
    }
}
