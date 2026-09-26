package pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.Function;

public class Constants {

    // Carried over directly from last year's MecanumConstants - motor names/directions are a
    // hardware wiring fact, not a tuned value, so they survive the Pedro Pathing rewrite as-is.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront"); // was leftFrontMotorName, "was leftBack"
        c.backLeftName.set("leftBack"); // was leftRearMotorName, "was leftFront"
        c.frontRightName.set("rightFront");
        c.backRightName.set("rightBack");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // Carried over directly from last year's PinpointConstants. xPodOffset/yPodOffset are the new
    // names for forwardPodY/strafePodX (same GoBildaPinpointDriver#setOffsets(x, y) call underneath).
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pp");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        c.xPodOffset.set(5.25); // was forwardPodY
        c.yPodOffset.set(-2.25 - 2.3824 - 2.5); // was strafePodX
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED); // was forwardEncoderDirection
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD); // was strafeEncoderDirection
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    /*
     * ============================== NEEDS RE-TUNING ==============================
     * Pedro Pathing replaced the old PIDF-based Follower (FollowerConstants: translational/
     * heading/drive PIDF, centripetalScaling, mass, forward/lateralZeroPowerAcceleration) with a
     * completely different feed-forward + braking-model algorithm called "Foresight". None of the
     * old PIDF gains translate mathematically into ForesightConfig - there is no valid conversion,
     * so they have NOT been carried over.
     *
     * The values below are placeholders (loosely seeded from last year's xVelocity/yVelocity and
     * forward/lateralZeroPowerAcceleration) that only exist so this compiles and the robot doesn't
     * throw at startup. They are NOT tuned.
     *
     * Before trusting this in autonomous, run the "Foresight Tuner" procedure from Tuning.java
     * (see pedro.procedures.ForesightTuner) on the actual robot and paste its generated
     * ForesightConfig code over this block.
     * ===============================================================================
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.forwardTranslational.set(Controller.proportional(0.055)); // was translationalPIDFCoefficients P
        c.strafeTranslational.set(Controller.proportional(0.055));
        c.headingFeedback.set(Controller.proportional(0.7)); // was headingPIDFCoefficients P

        c.coast.set(Controller.proportionalFeedforward(1.0)); // TODO: tune - no equivalent last year
        c.brake.set(Controller.proportionalFeedforward(1.0)); // TODO: tune - no equivalent last year

        c.headingBrakeCoefficients.set(Vector2D.cartesian(1.0, 0.0)); // TODO: tune - no equivalent last year
        c.linearBrakeCoefficients.set(Matrix.diag(25.583, 59.504313246199814)); // was |forward/lateralZeroPowerAcceleration|
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0, 0.0)); // TODO: tune - no equivalent last year

        c.maxAchievableForwardVelocity.set(90.0); // was xVelocity
        c.maxAchievableStrafeVelocity.set(70.0); // was yVelocity
        c.naturalForwardDeceleration.set(25.583); // was |forwardZeroPowerAcceleration|
        c.naturalStrafeDeceleration.set(59.504313246199814); // was |lateralZeroPowerAcceleration|
    });

    public static Function<HardwareMap, Drivetrain> drivetrainFunction = hardwareMap -> new Mecanum(hardwareMap, drivetrainConfig);
    public static Function<HardwareMap, Localizer> localizerFunction = hardwareMap -> new PinpointLocalizer(hardwareMap, localizerConfig);

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(localizerFunction.apply(hardwareMap), drivetrainFunction.apply(hardwareMap), new Foresight(foresightConfig));
    }
}
