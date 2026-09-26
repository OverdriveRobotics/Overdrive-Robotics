package pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import pedro.procedures.ForesightTuner;
import pedro.procedures.MecanumTuner;
import pedro.procedures.PinpointTuner;
import pedro.procedures.Tests;

public class Tuning {
    // Tuners go here - only the ones for the hardware this bot actually has (Mecanum + Pinpoint).
    // The other procedures (OTOS/OctoQuad/ThreeWheel/TwoWheel tuners) are for different localizer
    // hardware and don't apply here, so they're left unregistered.

    @Tuner(name = "Mecanum Tuner")
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner(name = "Pinpoint Tuner")
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner(name = "Foresight Tuner")
    public static Procedure foresightTuner() {
        return new ForesightTuner(Constants.localizerFunction, Constants.drivetrainFunction);
    }

    @Tuner(name = "Tests")
    public static Procedure tests() {
        return new Tests(Constants.drivetrainFunction, Constants.localizerFunction, () -> new Foresight(Constants.foresightConfig));
    }
}
