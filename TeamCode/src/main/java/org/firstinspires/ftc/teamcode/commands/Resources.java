package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Ivy requirement keys. Null-safe: a null robot/hardware yields a sentinel instead of an NPE in a constructor,
 * so the command is still built and then fails cleanly in start() with a useful message.
 */
final class Resources {
    private Resources() {}

    private static final Object NO_FLYWHEEL = new Object(), NO_STOPPER = new Object(),
            NO_FEED = new Object(), NO_TURRET = new Object();

    /** Every command that WRITES the shooter motors must require this. */
    static Object flywheel(RobotHardware r) { return r != null && r.shooterRight != null ? r.shooterRight : NO_FLYWHEEL; }
    static Object stopper(RobotHardware r) { return r != null && r.stopper != null ? r.stopper : NO_STOPPER; }
    /** The feed/intake motor: shared by intake and shooting so they cannot run together. */
    static Object feed(RobotHardware r) {
        return r != null && r.intakeAndTransferMotor != null ? r.intakeAndTransferMotor : NO_FEED;
    }
    static Object turret(RobotHardware r) { return r != null && r.turret != null ? r.turret : NO_TURRET; }
}
