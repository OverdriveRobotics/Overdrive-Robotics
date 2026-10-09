package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commands.CommandTests;
import org.firstinspires.ftc.teamcode.control.ControlTests;

/** Entry point: scripts/run_tests.sh. */
public final class AllTests {
    public static void main(String[] args) {
        ControlTests.run();
        CommandTests.run();
        System.out.println("passed " + T.passed + ", failed " + T.failed);
        for (String f : T.failures) System.out.println("  FAILED: " + f);
        System.exit(T.failed == 0 ? 0 : 1);
    }
}
