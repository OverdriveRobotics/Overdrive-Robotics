package org.firstinspires.ftc.teamcode.calibration;

/** A named, repeatable calibration procedure. {@link #paramKeys()} lists the registry keys it depends on (recorded in reports). */
public interface Procedure {
    String name();
    String[] paramKeys();
    ProcedureResult run(CalContext ctx);
}
