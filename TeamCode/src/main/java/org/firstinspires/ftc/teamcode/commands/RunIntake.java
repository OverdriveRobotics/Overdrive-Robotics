package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

import java.util.function.BooleanSupplier;

/**
 * Runs the intake/transfer motor. Owns only that motor; never touches the flywheel or stopper.
 * Finishes after durationMs (if &gt; 0) or when stopWhen returns true (if non-null); otherwise runs until cancelled.
 * Fails (and stops) on a detected jam when jam detection is enabled in {@link RobotConfig}.
 * No ball sensor exists on the robot, so "ball collected" is only available through the stopWhen condition.
 */
class RunIntake extends BaseCommand {
    private final RobotHardware robot;
    private final Storage.IntakeState mode;
    private final double power, durationMs;
    private final BooleanSupplier stopWhen;
    private double startMs, jamSinceMs;
    private boolean finished, ownsMotor;

    RunIntake(RobotHardware robot, Storage.IntakeState mode, double power, double durationMs, BooleanSupplier stopWhen) {
        super(0, ConflictBehavior.OVERRIDE, Resources.feed(robot));
        this.robot = robot;
        this.mode = mode;
        this.power = power;
        this.durationMs = durationMs;
        this.stopWhen = stopWhen;
    }

    @Override public void start() {
        finished = false;
        ownsMotor = false;
        jamSinceMs = -1;
        startMs = nowMs();
        String err = null;
        if (robot == null || robot.intakeAndTransferMotor == null) err = "intake hardware unavailable";
        else if (mode != Storage.IntakeState.INTAKING && mode != Storage.IntakeState.REVERSING)
            err = "invalid intake mode " + mode;
        else if (!Double.isFinite(power) || power <= 0 || power > 1) err = "invalid intake power " + power;
        else if (!Double.isFinite(durationMs)) err = "invalid duration " + durationMs;   // <= 0 (finite) = run until cancelled
        else if (Storage.shootInProgress) err = "feed motor owned by shooting";
        if (err != null) {
            finished = true;
            Storage.reportFailure("RunIntake", err);
            return;
        }
        robot.intakeAndTransferMotor.setPower(mode == Storage.IntakeState.INTAKING ? power : -power);
        ownsMotor = true;
        Storage.intakeState = mode;
    }

    @Override public void execute() {
        if (finished) return;
        double now = nowMs();
        if (RobotConfig.DETECT_INTAKE_JAM) {
            double amps = robot.intakeAndTransferMotor.getCurrent(CurrentUnit.AMPS);
            if (amps > RobotConfig.INTAKE_JAM_AMPS) {
                if (jamSinceMs < 0) jamSinceMs = now;
                if (now - jamSinceMs >= RobotConfig.INTAKE_JAM_MS) {
                    finished = true;
                    robot.intakeAndTransferMotor.setPower(0);
                    Storage.intakeState = Storage.IntakeState.JAMMED;
                    Storage.reportFailure("RunIntake", "intake jam detected (" + amps + " A)");
                }
            } else {
                jamSinceMs = -1;
            }
        }
        if (!finished && ((durationMs > 0 && now - startMs >= durationMs) || (stopWhen != null && stopWhen.getAsBoolean()))) {
            finished = true;
        }
    }

    @Override public boolean done() { return finished; }

    @Override public void end(EndCondition endCondition) {
        // A command that never started the motor (validation failure) must not touch it: shooting may own it.
        if (!ownsMotor || robot == null || robot.intakeAndTransferMotor == null) return;
        // If shooting took the feed motor from us (we were interrupted by it), it owns the motor and its state now.
        if (endCondition == EndCondition.INTERRUPTED && Storage.shootInProgress) return;
        robot.intakeAndTransferMotor.setPower(0);
        if (Storage.intakeState != Storage.IntakeState.JAMMED && Storage.intakeState != Storage.IntakeState.FEEDING)
            Storage.intakeState = Storage.IntakeState.IDLE;
    }
}
