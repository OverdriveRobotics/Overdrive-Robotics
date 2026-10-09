package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.Servo;


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
    public Servo hood;

    /** Battery voltage source (first voltage sensor on the hub), null if none was found. */
    public VoltageSensor batteryVoltage;

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
        Storage.invalidateHardwareState();

        // Accepted calibration (if any) overrides the compiled defaults. An invalid file is ignored as a whole.
        java.util.List<String> cfgProblems = org.firstinspires.ftc.teamcode.tuning.Tuning.applyActive(
                org.firstinspires.ftc.teamcode.tuning.Tuning.robotDir(
                        org.firstinspires.ftc.robotcore.internal.system.AppUtil.FIRST_FOLDER));
        if (!cfgProblems.isEmpty()) Storage.reportFailure("Config", String.join("; ", cfgProblems));

        // 1. Initialize Pedro Pathing
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(x,y,theta));


        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        intakeAndTransferMotor = hardwareMap.get(DcMotorEx.class, "intakeAndTransferMotor");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        stopper = hardwareMap.get(Servo.class, "stoper");
        hood = hardwareMap.get(Servo.class, "hood");
        for (VoltageSensor vs : hardwareMap.voltageSensor) { batteryVoltage = vs; break; }


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

        // Turret angle = (ticks - zero) converted by TurretUtil. The turret must be at TurretConfig.START_ANGLE_RAD here.
        Storage.turretZeroTicks = turret.getCurrentPosition();


    }


    /**
     * Updates the follower and stamps the pose time so readiness can reject a stale pose.
     * OpModes must call this instead of {@code follower.update()} once per loop.
     */
    public void updateLocalization() {
        follower.update();
        Storage.poseUpdateNanos = RobotClock.nanos();
    }

    /** Battery voltage, or NaN if unavailable (controllers treat NaN as a fault and output zero). */
    public double batteryVolts() {
        return batteryVoltage == null ? Double.NaN : batteryVoltage.getVoltage();
    }

    /**
     * Explicit safe shutdown. Scheduler.reset() only forgets commands: it does NOT call end() and does NOT touch
     * hardware, so OpMode stop paths must call this. Stops feed/intake/turret, closes the stopper and clears
     * shooting state. The flywheel is stopped only if {@code stopFlywheel} is true (per the OpMode's policy).
     */
    public void safeShutdown(boolean stopFlywheel) {
        if (intakeAndTransferMotor != null) intakeAndTransferMotor.setPower(0);
        if (turret != null) turret.setPower(0);
        if (stopper != null) stopper.setPosition(org.firstinspires.ftc.teamcode.commands.RobotConfig.STOPPER_CLOSED);
        Storage.shootInProgress = false;
        Storage.shootPhase = "IDLE";
        Storage.intakeState = Storage.IntakeState.IDLE;
        Storage.readyToShoot = false;
        if (stopFlywheel) {
            Storage.flywheelTargetVelocity = 0;
            Storage.flywheelReady = false;
            Storage.flywheelPower = 0;
            if (shooterLeft != null) { shooterLeft.setVelocity(0); shooterLeft.setPower(0); }
            if (shooterRight != null) { shooterRight.setVelocity(0); shooterRight.setPower(0); }
        }
    }
}
