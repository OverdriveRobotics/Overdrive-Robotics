package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.*;

import pedro.Constants;

/**
 * Pure Hardware Mapping Class
 * Contains hardware references, initializations, and Pedro Pathing instance.
 * Does not extend OpMode or CommandOpMode.
 */
public class RobotHardware {

    /* Pedro Pathing Follower */
    public Follower follower;

    /* Actuators & Motors */
    public DcMotorEx liftMotor;
    public DcMotorEx intakeMotor;

    /* Local reference to hardwareMap */
    private HardwareMap hardwareMap;

    /**
     * Default constructor
     */
    public RobotHardware() {}

    /**
     * Initializes all hardware components using the OpMode's HardwareMap.
     * @param hwMap The HardwareMap supplied by the OpMode
     */
    public void init(HardwareMap hwMap, double x, double y, double theta ) {
        this.hardwareMap = hwMap;

        // 1. Initialize Pedro Pathing
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(x,y,theta));



    }

}