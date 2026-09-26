package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import pedro.Constants;

@TeleOp(name = "***LAST CALL BLUE NEAR TELEOP")
public class teleop extends OpMode {

    private Follower follower;

    // drivetrain
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // flywheel
    private DcMotorEx flywheelLeader;
    private DcMotorEx flywheelFollower;
    private DcMotorEx intakeMotor;

    private double targetVelocity = 1500;

    // turret
    private DcMotorEx turret;

    // 🔴 turret override
    private boolean turretManualMode = false;
    private boolean turretToggleLast = false;
    private double turretManualSpeed = 0.25;

    private double goalx = 15;
    private double goaly = 130;

    private Servo ServoStopper;
    private final double OPEN = .260;
    private final double CLOSE = 0.05;

    private boolean flywheelOn = false;
    private boolean flywheelButtonLast = false;

    // Shoot state machine
    private boolean shooting = false;
    private long shootStartTime = 0;

    private final long SHOOT_DURATION = 1500;
    private final long SERVO_LEAD_TIME = 150;
    private final long SPIN_UP_TIMEOUT = 1500; // fire anyway if flywheel never reaches speed
    private final double VEL_TOLERANCE = 40;   // ticks/sec

    private long spinUpStartTime = 0;

    @Override
    public void init() {

        flywheelLeader = hardwareMap.get(DcMotorEx.class, "rightShooter");
        flywheelFollower = hardwareMap.get(DcMotorEx.class, "leftShooter");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        flywheelLeader.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(162.4, 0, 0, 9.3));
        flywheelFollower.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(162.4, 0, 0, 9.3));

        frontLeft  = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft   = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight  = hardwareMap.get(DcMotorEx.class, "rightBack");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        ServoStopper = hardwareMap.get(Servo.class, "block");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelLeader.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelFollower.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelLeader.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelFollower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Coast when stopped so the flywheel keeps its speed instead of braking to zero
        flywheelLeader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelFollower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        ServoStopper.setPosition(CLOSE);


        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(50, 88, Math.toRadians(180)));
        follower.update();

        telemetry.addLine("System Initialized");
    }

    @Override
    public void loop() {

        follower.update();

        double ex = follower.pose().x();
        double ey = follower.pose().y();

        double dist = Math.sqrt(Math.pow(ex - goalx, 2) + Math.pow(ey - goaly, 2));

        double leadingTerm = 0.066883 * Math.pow(dist, 2);
        double nextTerm = -5.29179 * dist;
        double lastTerm = 1581.1435;

        targetVelocity = leadingTerm + nextTerm + lastTerm;

        // 🔴 turret control
        handleTurretControl();

        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", flywheelLeader.getVelocity());
        telemetry.addData("distance", dist);

        handleFlywheelToggle();
        handleShoot();
        handleIntakeManual();

        double y  = -gamepad1.left_stick_y * 0.8;
        double x  =  gamepad1.left_stick_x * 0.8;
        double rx =  gamepad1.right_stick_x * 0.8;

        driveRobotCentric(y, x, rx);

        telemetry.update();
    }

    // ================= TURTLE CONTROL =================

    private void handleTurretControl() {

        boolean togglePressed = gamepad2.y;

        if (togglePressed && !turretToggleLast) {
            turretManualMode = !turretManualMode;
        }
        turretToggleLast = togglePressed;

        if (turretManualMode) {

            double power = 0;

            if (gamepad2.dpad_left) {
                power = turretManualSpeed;
            }
            else if (gamepad2.dpad_right) {
                power = -turretManualSpeed;
            }

            turret.setPower(power);
            telemetry.addLine("Turret: MANUAL (DPAD)");

        }
    }

    // ================= FLYWHEEL =================

    private void handleFlywheelToggle() {
        if (shooting) return;

        boolean buttonPressed = gamepad2.a;

        if (buttonPressed && !flywheelButtonLast) {
            flywheelOn = !flywheelOn;
        }

        flywheelButtonLast = buttonPressed;

        if (flywheelOn) spinFlywheel();
        else stopFlywheel();
    }

    private void handleShoot() {

        if (gamepad1.b && !shooting) {
            shooting = true;
            flywheelOn = true;
            spinUpStartTime = System.currentTimeMillis();
            shootStartTime = 0; // 0 = still waiting for flywheel to reach speed
        }

        if (!shooting) return;

        spinFlywheel();

        if (shootStartTime == 0) {
            boolean timedOut = System.currentTimeMillis() - spinUpStartTime >= SPIN_UP_TIMEOUT;
            if (!atSpeed() && !timedOut) {
                intakeMotor.setPower(0);
                return;
            }
            shootStartTime = System.currentTimeMillis();
            ServoStopper.setPosition(OPEN);
        }

        long elapsed = System.currentTimeMillis() - shootStartTime;

        if (elapsed >= SERVO_LEAD_TIME && elapsed < SHOOT_DURATION) {
            intakeMotor.setPower(-1);
        }

        if (elapsed >= SHOOT_DURATION) {
            intakeMotor.setPower(0);
            ServoStopper.setPosition(CLOSE);
            shooting = false; // flywheel keeps spinning; gamepad2 toggle turns it off
        }
    }

    private boolean atSpeed() {
        return Math.abs(flywheelLeader.getVelocity() - targetVelocity) < VEL_TOLERANCE;
    }

    private void handleIntakeManual() {

        if (shooting) return;

        double intakeIn = gamepad1.left_trigger;
        double intakeOut = gamepad1.right_trigger;

        if (intakeIn > 0.05) intakeMotor.setPower(intakeIn);
        else if (intakeOut > 0.05) intakeMotor.setPower(-intakeOut);
        else intakeMotor.setPower(0);
    }

    // ================= DRIVE =================

    private void driveRobotCentric(double y, double x, double rx) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        frontLeft.setPower((y + x + rx) / denominator);
        backLeft.setPower((y - x + rx) / denominator);
        frontRight.setPower((y - x - rx) / denominator);
        backRight.setPower((y + x - rx) / denominator);
    }

    private void spinFlywheel() {
        flywheelLeader.setVelocity(targetVelocity);
        flywheelFollower.setVelocity(targetVelocity);
    }

    private void stopFlywheel() {
        flywheelLeader.setPower(0);
        flywheelFollower.setPower(0);
    }
}
