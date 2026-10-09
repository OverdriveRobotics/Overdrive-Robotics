package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.behaviors.ConflictBehavior;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

/**
 * Explicit flywheel stop. Priority 10 on the flywheel resource, so it interrupts the regulator (the regulator's
 * end() does not restart anything). Clears the target and readiness; deliberately does NOT clear
 * {@code Storage.flywheelFault} (diagnostics survive; the next spin-up request resets it).
 */
class StopFlywheel extends BaseCommand {
    private final RobotHardware robot;

    StopFlywheel(RobotHardware robot) {
        super(10, ConflictBehavior.OVERRIDE, FlywheelUtil.flywheelKey(robot));
        this.robot = robot;
    }

    @Override public void start() {
        Storage.flywheelTargetVelocity = 0;
        Storage.flywheelReady = false;
        Storage.flywheelStableForMs = 0;
        Storage.flywheelPower = 0;
        Storage.flywheelMode = "off";
        if (FlywheelUtil.hardwareAvailable(robot)) {
            robot.shooterLeft.setVelocity(0);
            robot.shooterRight.setVelocity(0);
            robot.shooterLeft.setPower(0);
            robot.shooterRight.setPower(0);
        }
    }

    @Override public boolean done() { return true; }
}
