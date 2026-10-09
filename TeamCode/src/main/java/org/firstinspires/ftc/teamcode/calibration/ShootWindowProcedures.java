package org.firstinspires.ftc.teamcode.calibration;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.commands.RobotConfig;
import org.firstinspires.ftc.teamcode.commands.ShootStatus;
import org.firstinspires.ftc.teamcode.control.ShootingReadiness;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;
import org.firstinspires.ftc.teamcode.tuning.Metrics;
import org.firstinspires.ftc.teamcode.tuning.RunLog;
import org.firstinspires.ftc.teamcode.tuning.WindowTracker;

import java.util.Map;

/** Live inspection of the shooting window: which conditions hold, when the window opens/closes, shot outcomes. */
public final class ShootWindowProcedures {
    private ShootWindowProcedures() {}

    static final String[] KEYS = {"robot.REQUIRE_LOCALIZATION", "robot.REQUIRE_TURRET_ALIGNED", "robot.SHOOT_ZONE_ENABLED",
            "robot.SHOOT_ZONE_MIN_X", "robot.SHOOT_ZONE_MAX_X", "robot.SHOOT_ZONE_MIN_Y", "robot.SHOOT_ZONE_MAX_Y",
            "robot.MAX_SHOOT_SPEED", "turret.ALIGN_TOLERANCE_RAD", "turret.ALIGN_VELOCITY_TOL_RAD_S", "turret.MAX_POSE_AGE_MS",
            "robot.FLYWHEEL_TOLERANCE", "robot.FLYWHEEL_STABLE_MS", "robot.READY_TIMEOUT_MS", "robot.SHOT_TIMEOUT_MS",
            "robot.RECOVER_TIMEOUT_MS", "robot.SHOT_DROP_THRESHOLD", "robot.STOPPER_SETTLE_MS"};

    static int phaseCode(String p) {
        switch (p) {
            case "WAIT_READY": return 1; case "OPENING": return 2; case "FEED": return 3;
            case "CLOSING": return 4; case "RECOVER": return 5; case "FAILED": return -1; default: return 0;
        }
    }

    /** @param balls 0 = observe only (never feeds). The robot may be hand-moved or driven by the OpMode's operator. */
    public static Procedure monitor(final double seconds, final int balls) {
        return new Procedure() {
            @Override public String name() { return balls > 0 ? "window-shoot" : "window-monitor"; }
            @Override public String[] paramKeys() { return KEYS; }
            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.FLYWHEEL, Preflight.Need.BATTERY, Preflight.Need.LOCALIZATION);
                if (balls > 0) pre.addAll(Preflight.check(r, Preflight.Need.FEED, Preflight.Need.STOPPER));
                if (balls < 0 || balls > RobotConfig.MAX_BALLS) pre.add("balls must be 0.." + RobotConfig.MAX_BALLS);
                if (!(seconds >= 2 && seconds <= 90)) pre.add("duration must be 2..90 s");
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);
                ProcedureResult out = new ProcedureResult();
                RunLog log = new RunLog("t", "x", "y", "heading", "speed", "ready", "flywheel_ready", "turret_err", "fly_meas", "fly_target", "phase", "shots");
                out.log = log;
                if (balls > 0 && !c.confirm("Load " + balls + " ball(s) and keep the shot path CLEAR. The robot will shoot when the window opens. A=go")) { out.aborted = true; return out; }
                double v = Storage.activeTargetIndex == 0 ? RobotConfig.FLYWHEEL_VELOCITY_A : RobotConfig.FLYWHEEL_VELOCITY_B;
                WindowTracker tracker = new WindowTracker();
                Command aim = RobotCommands.aimTurret(r), mon = RobotCommands.readinessMonitor(r);
                Command spin = RobotCommands.spinUpFlywheel(r, v), shoot = null;
                try {
                    RobotCommands.ensureFlywheelRegulator(r);
                    Scheduler.schedule(aim, mon, spin);
                    if (balls > 0) { shoot = RobotCommands.shoot(r, balls, v, RobotConfig.READY_TIMEOUT_MS); Scheduler.schedule(shoot); }
                    double t0 = c.timeSec();
                    final int shotBase = Storage.totalBallsShot;
                    int lastShots = 0;
                    String lastFail = Storage.lastFailure;
                    double firstShotT = Double.NaN;
                    int shotsSeen = 0;
                    while (!c.abortRequested() && c.timeSec() - t0 < seconds) {
                        c.tick();
                        double t = c.timeSec() - t0;
                        ShootingReadiness.Result res = ShootStatus.evaluate(r, !Storage.shootInProgress, "shot in progress");
                        // while a shot is in progress "feed busy" is expected; the window itself is what we track
                        boolean inWindow = res.ready || (Storage.shootInProgress && "shot in progress".equals(res.reason));
                        tracker.sample(t, inWindow, res.reason);
                        Pose p = r.follower.pose(); Velocity vel = r.follower.velocity();
                        if (Storage.totalBallsShot - shotBase > lastShots) { lastShots = Storage.totalBallsShot - shotBase; shotsSeen = lastShots; if (Double.isNaN(firstShotT)) firstShotT = t; }
                        log.add(t, p.x(), p.y(), p.heading(), Math.hypot(vel.vx, vel.vy), inWindow ? 1 : 0, Storage.flywheelReady ? 1 : 0,
                                Storage.turretError, Storage.flywheelMeasuredVelocity, Storage.flywheelTargetVelocity, phaseCode(Storage.shootPhase), shotsSeen);
                        if (shoot != null && !Scheduler.isScheduled(shoot)) break;
                    }
                    out.aborted = c.abortRequested();
                    out.metrics.putAll(tracker.metrics());
                    if (balls > 0) {
                        out.metrics.put("shots_ok", (double) shotsSeen);
                        out.metrics.put("shot_success_rate", (double) shotsSeen / balls);
                        out.metrics.put("first_shot_after_s", firstShotT);
                    }
                    for (Map.Entry<String, Double> e : tracker.blockedReasons().entrySet())
                        out.notes.add(String.format("blocked %.2f s: %s", e.getValue(), e.getKey()));
                    if (Storage.lastFailure != null && !Storage.lastFailure.equals(lastFail)) out.notes.add("failure: " + Storage.lastFailure);
                } finally {
                    if (shoot != null) Scheduler.cancel(shoot);
                    Scheduler.cancel(aim); Scheduler.cancel(mon); Scheduler.cancel(spin);
                    Scheduler.schedule(RobotCommands.stopFlywheel(r));
                    c.tick();
                    r.safeShutdown(false);
                }
                return out;
            }
        };
    }

    @SuppressWarnings("unused") private static double unused() { return Metrics.mean(new double[]{0}); }
}
