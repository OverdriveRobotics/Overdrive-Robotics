package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Reusable autonomous building blocks written with Ivy compositions (see docs/IVY_GUIDE.md). Each call returns a
 * fresh command. They assume {@link RobotCommands#startShooterSystems} is running (regulator, turret aim, readiness).
 */
public final class AutoRoutines {
    private AutoRoutines() {}

    /** Drive {@code path} while spinning up and shooting; hold the final pose afterwards. */
    public static Command shootWhileDriving(RobotHardware robot, Path path, int balls, double velocity, double windowMs) {
        return sequential(RobotCommands.movingShot(robot, path, balls, velocity, windowMs), hold(robot.follower));
    }

    /**
     * One scoring cycle: drive to the pickup with the intake running (intake is cancelled - and its end() stops the
     * motor - the moment the path finishes, because the path is the DEADLINE), then drive back shooting on the move.
     */
    public static Command cycle(RobotHardware robot, Path toPickup, Path toShoot, int balls, double velocity, double windowMs) {
        return sequential(
                deadline(follow(robot.follower, toPickup), RobotCommands.runIntake(robot, false)),
                shootWhileDriving(robot, toShoot, balls, velocity, windowMs));
    }
}
