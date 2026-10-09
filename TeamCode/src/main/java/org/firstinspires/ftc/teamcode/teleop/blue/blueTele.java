package org.firstinspires.ftc.teamcode.teleop.blue;

import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.commands.RobotConfig;
import org.firstinspires.ftc.teamcode.commands.RobotTelemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

/**
 * Gamepad 1: sticks drive (Pedro drive-or-hold, unchanged).
 *  right bumper : shoot 3 balls at the active target's velocity (waits for the live readiness window)
 *  left bumper  : switch between target A and target B (turret re-aims, readiness re-evaluates)
 *  A / B        : spin up / stop the flywheel      X : intake on/off      Y : reverse intake on/off
 */
@TeleOp(name = "Robot-Centric TeleOp (Drive or Hold)")
public class blueTele extends OpMode {

    private final RobotHardware robot = new RobotHardware();
    private final Pose autoendpose = Storage.autoEndPose;

    private Command intake;
    private boolean lastRb, lastLb, lastA, lastB, lastX, lastY;

    @Override
    public void init() {
        robot.init(hardwareMap, autoendpose.x(), autoendpose.y(), autoendpose.heading());
        // Ivy's scheduler is static: clear anything left over from a previous OpMode.
        Scheduler.reset();
    }

    @Override
    public void start() {
        // Background loops (distinct resources, so they run alongside driving and shooting):
        // flywheel regulator, turret aiming, readiness monitor.
        RobotCommands.startShooterSystems(robot);
    }

    @Override
    public void loop() {
        // Read raw joystick inputs
        double forward = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double turn    = gamepad1.right_stick_x;

        // Robot-Centric drive with automatic position holding on stick release
        ManualDrive.driveOrHold(robot.follower, forward, lateral, turn);

        // Required every cycle (also stamps the pose time that readiness uses to reject stale localization)
        robot.updateLocalization();

        boolean rb = gamepad1.right_bumper, lb = gamepad1.left_bumper;
        boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, y = gamepad1.y;

        if (lb && !lastLb) Storage.toggleTarget();
        if (rb && !lastRb) Scheduler.schedule(RobotCommands.shoot(robot, RobotConfig.MAX_BALLS, activeVelocity()));
        if (a && !lastA) Scheduler.schedule(RobotCommands.spinUpFlywheel(robot, activeVelocity()));
        if (b && !lastB) Scheduler.schedule(RobotCommands.stopFlywheel(robot));
        if (x && !lastX) toggleIntake(false);
        if (y && !lastY) toggleIntake(true);
        lastRb = rb; lastLb = lb; lastA = a; lastB = b; lastX = x; lastY = y;

        Scheduler.execute();

        RobotTelemetry.add(telemetry, robot);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Scheduler.reset() does not call end() or touch hardware, so shut down explicitly.
        Scheduler.reset();
        robot.safeShutdown(true);   // OpMode-end policy: the SDK stops the motors anyway, so stop the flywheel state too
    }

    private double activeVelocity() {
        return Storage.activeTargetIndex == 0 ? RobotConfig.FLYWHEEL_VELOCITY_A : RobotConfig.FLYWHEEL_VELOCITY_B;
    }

    private void toggleIntake(boolean reverse) {
        if (intake != null && Scheduler.isScheduled(intake)) {
            Scheduler.cancel(intake);   // end() stops the motor
            intake = null;
        } else {
            intake = RobotCommands.runIntake(robot, reverse);
            Scheduler.schedule(intake);
        }
    }
}
