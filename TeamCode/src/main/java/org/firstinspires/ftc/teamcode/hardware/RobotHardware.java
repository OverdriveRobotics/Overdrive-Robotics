package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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

    /* Motors */
    public DcMotorEx intakeAndTransferMotor;
    public DcMotorEx shooterRight;
    public DcMotorEx shooterLeft;
    public DcMotorEx turret;

    public Servo stopper;

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


        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        intakeAndTransferMotor = hardwareMap.get(DcMotorEx.class, "intakeAndTransferMotor");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        stopper = hardwareMap.get(Servo.class, "stoper");


        shooterRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        shooterRight.setDirection(DcMotorEx.Direction.FORWARD);
        shooterLeft.setDirection(DcMotorEx.Direction.REVERSE);
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        intakeAndTransferMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients shooterR = new PIDFCoefficients(1,1,1,1);
        PIDFCoefficients shooterL = new PIDFCoefficients(1,1,1,1);
        PIDFCoefficients tur = new PIDFCoefficients(1,1,1,1);


        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeAndTransferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        shooterRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterR);
        shooterLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterL);
        turret.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, tur);


    }

}