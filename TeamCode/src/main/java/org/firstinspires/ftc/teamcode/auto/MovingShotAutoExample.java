package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.AutoRoutines;
import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.commands.RobotConfig;
import org.firstinspires.ftc.teamcode.commands.RobotTelemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;

/**
 * EXAMPLE ONLY (@Disabled): shows the intended concurrent structure. Every pose and target below is a PLACEHOLDER,
 * not a field measurement: replace them before enabling.
 *
 * While the robot follows the path the flywheel spins up, the turret tracks the active target, readiness is
 * evaluated live and the shot starts as soon as the window is valid - it does not wait for the path to finish.
 * If the window never occurs the shot gives up after the per-ball timeout while the path continues.
 */
@Disabled
@Autonomous(name = "EXAMPLE Moving Shot")
public class MovingShotAutoExample extends OpMode {
    private final RobotHardware robot = new RobotHardware();

    private static final Pose START = new Pose(0, 0, 0);            // PLACEHOLDER
    private static final Pose SHOOT_END = new Pose(48, 0, 0);       // PLACEHOLDER

    @Override
    public void init() {
        robot.init(hardwareMap, START.x(), START.y(), START.heading());
        Scheduler.reset();
        Storage.setTarget(0, 72, 72);        // PLACEHOLDER target A
        Storage.setTarget(1, 72, -72);       // PLACEHOLDER target B
        Storage.selectTarget(0);
    }

    @Override
    public void start() {
        RobotCommands.startShooterSystems(robot);
        Path toShoot = line(START, SHOOT_END).constant(0);
        Scheduler.schedule(AutoRoutines.shootWhileDriving(
                robot, toShoot, RobotConfig.MAX_BALLS, RobotConfig.FLYWHEEL_VELOCITY_A, 5000));
    }

    @Override
    public void loop() {
        robot.updateLocalization();
        Scheduler.execute();
        RobotTelemetry.add(telemetry, robot);
        telemetry.update();
    }

    @Override
    public void stop() {
        Storage.autoEndPose = robot.follower.pose();   // read by teleop's init
        Scheduler.reset();
        robot.safeShutdown(true);
    }
}
