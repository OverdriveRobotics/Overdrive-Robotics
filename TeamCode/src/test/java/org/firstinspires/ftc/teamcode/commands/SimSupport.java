package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.near;
import static org.firstinspires.ftc.teamcode.T.test;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.hardware.Storage;

/** Shared helpers for the simulation suites. */
public final class SimSupport {
    private SimSupport() {}

    public interface Body { void run(SimRobot s) throws Exception; }

    public static void sim(String name, Body b) {
        test(name, () -> {
            SimRobot s = SimRobot.fresh();
            try { b.run(s); } finally { s.cleanup(); }
        });
    }

    public static boolean running(Command c) { return Scheduler.isScheduled(c); }

    public static Command spinUpReady(SimRobot s, double target) {
        Command c = RobotCommands.spinUpFlywheel(s.robot, target);
        Scheduler.schedule(c);
        check(s.runUntil(() -> !running(c), 6000), "spin-up to " + target + " should finish");
        return c;
    }

    public static void assertSafe(SimRobot s, String why) {
        check(s.intake.power == 0, why + ": feed motor stopped");
        near(RobotConfig.STOPPER_CLOSED, s.stopper.position, 0, why + ": stopper closed");
        check(!Storage.shootInProgress, why + ": shootInProgress cleared");
    }
}
