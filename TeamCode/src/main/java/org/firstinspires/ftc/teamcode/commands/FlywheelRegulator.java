package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.control.FlywheelConfig;
import org.firstinspires.ftc.teamcode.control.FlywheelLqrController;
import org.firstinspires.ftc.teamcode.hardware.RobotClock;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

/**
 * THE single writer of the shooter motors while the flywheel is running, and the single owner of flywheel
 * readiness. Runs continuously (never finishes by itself) so velocity is regulated and readiness is re-evaluated
 * from the live measurement every scheduler loop, whether or not a spin-up command is still active.
 *
 * Two output paths behind {@link FlywheelConfig#MODEL_IDENTIFIED}:
 *  - false (default): the SDK's built-in velocity PIDF via setVelocity(), as the project did before;
 *  - true: {@link FlywheelLqrController} -> setPower(volts / batteryVoltage) in RUN_WITHOUT_ENCODER.
 * Both motors receive the same command; their measured velocities are averaged (per-side values are not controlled
 * independently, which assumes both motors drive one flywheel).
 *
 * Priority 5 / CANCEL: a second regulator cannot displace a running one; {@link StopFlywheel} (priority 10) can.
 */
class FlywheelRegulator extends BaseCommand {
    private final RobotHardware robot;
    private FlywheelLqrController lqr;
    private double appliedTarget = -1, inBandSinceMs = -1, lastMs;
    private boolean faultHandled, failedToStart;

    FlywheelRegulator(RobotHardware robot) {
        super(5, ConflictBehavior.CANCEL, FlywheelUtil.flywheelKey(robot));
        this.robot = robot;
    }

    @Override public void start() {
        failedToStart = false;
        appliedTarget = -1;
        inBandSinceMs = -1;
        lastMs = nowMs();
        faultHandled = false;
        if (!FlywheelUtil.hardwareAvailable(robot)) {
            failedToStart = true;
            fault("flywheel hardware unavailable");
            return;
        }
        if (FlywheelConfig.MODEL_IDENTIFIED) {
            try {
                lqr = new FlywheelLqrController(FlywheelLqrController.Params.fromConfig());
            } catch (RuntimeException e) {
                failedToStart = true;
                fault("LQR design failed: " + e.getMessage());
                return;
            }
            robot.shooterLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            robot.shooterRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            Storage.flywheelMode = "LQR";
        } else {
            Storage.flywheelMode = "SDK-PIDF";
        }
    }

    @Override public void execute() {
        if (failedToStart) return;
        double now = nowMs();
        double dt = Math.max(1e-3, (now - lastMs) / 1000.0);
        lastMs = now;

        double target = Storage.flywheelTargetVelocity;
        double v = FlywheelUtil.sample(robot);   // publishes measurement + timestamp

        if (Storage.flywheelFault != null) {      // faulted: hold the motors off until a new request clears it
            if (!faultHandled) { writePower(0); faultHandled = true; }
            inBandSinceMs = -1;
            Storage.flywheelStableForMs = 0;
            Storage.flywheelReady = false;
            return;
        }
        if (faultHandled) {                        // fault was cleared by a new request: restart cleanly
            faultHandled = false;
            appliedTarget = -1;
            if (lqr != null) lqr.reset();
        }
        if (!Double.isFinite(v)) {
            fault("invalid flywheel velocity reading");
            return;
        }

        // ---- readiness from the live measurement (never from a remembered flag) ----
        if (FlywheelUtil.inTolerance(v, target, Storage.flywheelTolerance)) {
            if (inBandSinceMs < 0) inBandSinceMs = now;
        } else {
            inBandSinceMs = -1;
        }
        double stableFor = inBandSinceMs < 0 ? 0 : now - inBandSinceMs;
        Storage.flywheelStableForMs = stableFor;
        Storage.flywheelReady = target > 0 && stableFor >= Storage.flywheelStableMs;

        // ---- output ----
        if (lqr != null) {
            if (target <= 0 || target > RobotConfig.MAX_FLYWHEEL_VELOCITY) {
                writePower(0);
                lqr.reset();
                appliedTarget = target;
                return;
            }
            double p = lqr.update(target, v, robot.batteryVolts(), dt);
            if (lqr.hasFault()) {
                writePower(0);
                fault("flywheel controller fault (battery voltage or velocity invalid)");
                return;
            }
            writePower(p);
        } else if (target != appliedTarget) {
            appliedTarget = target;
            if (target > 0 && target <= RobotConfig.MAX_FLYWHEEL_VELOCITY) {
                robot.shooterLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                robot.shooterRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                robot.shooterLeft.setVelocity(target);
                robot.shooterRight.setVelocity(target);
            } else {
                robot.shooterLeft.setVelocity(0);
                robot.shooterRight.setVelocity(0);
            }
        }
    }

    @Override public boolean done() { return false; }

    @Override public void end(EndCondition endCondition) {
        // A power-commanded flywheel must not keep a stale power with nobody regulating it; the SDK PIDF path
        // holds its velocity on its own, so it is left running (StopFlywheel / safeShutdown stop it explicitly).
        if (lqr != null && FlywheelUtil.hardwareAvailable(robot)) writePower(0);
        Storage.flywheelReady = false;
        Storage.flywheelPower = 0;
    }

    private void writePower(double p) {
        robot.shooterLeft.setPower(p);
        robot.shooterRight.setPower(p);
        Storage.flywheelPower = p;
    }

    private void fault(String msg) {
        Storage.flywheelFault = msg;
        Storage.flywheelReady = false;
        Storage.reportFailure("FlywheelRegulator", msg);
        if (FlywheelUtil.hardwareAvailable(robot)) writePower(0);
        faultHandled = true;
    }
}
