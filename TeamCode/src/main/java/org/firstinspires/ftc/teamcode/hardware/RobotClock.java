package org.firstinspires.ftc.teamcode.hardware;

import java.util.function.LongSupplier;

/** Single time source for commands and Storage so tests can drive time deterministically. */
public final class RobotClock {
    private RobotClock() {}

    private static volatile LongSupplier source = System::nanoTime;

    public static long nanos() { return source.getAsLong(); }
    public static double ms() { return source.getAsLong() / 1e6; }

    /** Test hook. Pass null to restore the real clock. */
    public static void setSource(LongSupplier s) { source = s != null ? s : System::nanoTime; }
}
