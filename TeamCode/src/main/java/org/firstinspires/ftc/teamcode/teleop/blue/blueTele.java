package org.firstinspires.ftc.teamcode.teleop.blue;

import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

@TeleOp(name = "Robot-Centric TeleOp (Drive or Hold)")
public class blueTele extends OpMode {

    private final RobotHardware robot = new RobotHardware();
    private final Pose autoendpose = Storage.autoEndPose;

    @Override
    public void init() {
        robot.init(hardwareMap, autoendpose.x(), autoendpose.y(), autoendpose.heading());
    }

    @Override
    public void loop() {
        // Read raw joystick inputs
        double forward = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double turn    = gamepad1.right_stick_x;

        // Robot-Centric drive with automatic position holding on stick release
        ManualDrive.driveOrHold(
                robot.follower,
                forward,
                lateral,
                turn
        );

        // Required: Update follower every cycle
        robot.follower.update();







    }
}