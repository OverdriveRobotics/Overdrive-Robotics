package org.firstinspires.ftc.teamcode.calibration;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Pure decision logic of the turret procedures (unit-tested without hardware). */
public final class TurretCalLogic {
    private TurretCalLogic() {}

    /**
     * Encoder direction. Convention: angle = ENCODER_SIGN * ticks must INCREASE with positive motor power, and
     * positive angle is counter-clockwise seen from above. If positive power turns the turret clockwise the MOTOR
     * direction must be reversed in RobotHardware (source, not a tunable), so no sign is suggested.
     */
    public static Map<String, Double> directionSuggestion(int deltaTicks, boolean movedCounterClockwise, List<String> notes) {
        Map<String, Double> s = new LinkedHashMap<>();
        if (Math.abs(deltaTicks) < 5) { notes.add("encoder barely moved (" + deltaTicks + " ticks): check the encoder cable/port and that the turret actually turned"); return s; }
        if (!movedCounterClockwise) {
            notes.add("positive power turned the turret CLOCKWISE: reverse turret.setDirection(...) in RobotHardware.init, then repeat this test (not a tunable value)");
            return s;
        }
        s.put("turret.ENCODER_SIGN", deltaTicks > 0 ? 1.0 : -1.0);
        return s;
    }

    /**
     * Ticks per turret revolution from hand-rotations by a known angle. Returns {mean, stddev} of the per-repeat
     * estimates (the spread is the encoder consistency).
     */
    public static double[] ticksPerTurretRev(double[] deltaTicks, double knownAngleDeg) {
        double[] est = new double[deltaTicks.length];
        for (int i = 0; i < est.length; i++) est[i] = Math.abs(deltaTicks[i]) / knownAngleDeg * 360.0;
        return new double[]{org.firstinspires.ftc.teamcode.tuning.Metrics.mean(est), org.firstinspires.ftc.teamcode.tuning.Metrics.stddev(est)};
    }

    /**
     * EXTERNAL reduction implied by a measured ticks-per-turret-rev and the encoder's ticks per shaft rev. Using the
     * SHAFT figure here (not an output-shaft figure) is what keeps the internal gearbox from being counted twice.
     */
    public static double externalRatio(double ticksPerTurretRev, double shaftTicksPerRev) {
        return ticksPerTurretRev / shaftTicksPerRev;
    }
}
