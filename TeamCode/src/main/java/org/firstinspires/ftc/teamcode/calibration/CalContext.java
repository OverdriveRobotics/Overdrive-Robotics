package org.firstinspires.ftc.teamcode.calibration;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Everything a calibration procedure needs from its environment. The same procedure code runs against the real
 * robot (LinearOpMode implementation) and against the simulator (test source set), so measurements, metrics and
 * candidate generation are identical in both; only the {@link #mode()} label differs.
 */
public interface CalContext {
    RobotHardware robot();

    /** "HARDWARE" or "SIMULATION": recorded in every report and shown next to every result. */
    String mode();

    /** One loop iteration (named tick() because LinearOpMode.loop() is final): updateLocalization, Scheduler.execute, operator I/O, idle. Procedures call this in their loops. */
    void tick();

    /** Seconds on the robot clock (monotonic within a run). */
    double timeSec();

    /** True after the operator pressed ABORT (or the OpMode is stopping). Procedures must check it every loop. */
    boolean abortRequested();

    /** Blocks (still calling {@link #loop()}) until the operator answers; false for "no" or abort. */
    boolean confirm(String prompt);

    /** Operator-entered number (tape-measure distance, tachometer RPM, known angle...). */
    double promptValue(String prompt, double initial, double step, double min, double max);

    /** Free-text progress line shown on the driver station (and printed in simulation). */
    void status(String line);
}
