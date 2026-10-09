package org.firstinspires.ftc.teamcode.calibration;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.commands.AutoRoutines;
import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.commands.RobotConfig;
import org.firstinspires.ftc.teamcode.commands.ShootStatus;
import org.firstinspires.ftc.teamcode.control.ShootingReadiness;
import org.firstinspires.ftc.teamcode.control.TurretConfig;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;
import org.firstinspires.ftc.teamcode.tuning.Metrics;
import org.firstinspires.ftc.teamcode.tuning.RunLog;
import org.firstinspires.ftc.teamcode.tuning.WindowTracker;

/**
 * Staged moving-shot validation. Stages escalate: 1 stationary flywheel+turret, 2 stationary shooting, 3 path while
 * spinning up (no feed), 4 one moving shot, 5 multi-ball moving shot, 6 a full scoring cycle. The production
 * commands (RobotCommands / AutoRoutines) are what actually run; this class only supervises and records.
 */
public final class MovingShotProcedures {
    private MovingShotProcedures() {}

    public static final String[] STAGE_NAMES = {"", "1 stationary flywheel+turret", "2 stationary shooting", "3 path + spin-up (no feed)",
            "4 moving shot (1 ball)", "5 moving multi-ball", "6 full scoring cycle"};

    public static Procedure stage(final int stage, final Pose start, final Pose end, final int balls, final double windowMs, final double maxSeconds) {
        return new Procedure() {
            @Override public String name() { return "movingshot-stage" + stage; }
            @Override public String[] paramKeys() { return ShootWindowProcedures.KEYS; }

            @Override public ProcedureResult run(CalContext c) {
                RobotHardware r = c.robot();
                java.util.List<String> pre = Preflight.check(r, Preflight.Need.FLYWHEEL, Preflight.Need.BATTERY, Preflight.Need.LOCALIZATION, Preflight.Need.TURRET);
                if (stage >= 2) pre.addAll(Preflight.check(r, Preflight.Need.FEED, Preflight.Need.STOPPER));
                if (stage < 1 || stage > 6) pre.add("stage must be 1..6");
                if (balls < 1 || balls > RobotConfig.MAX_BALLS) pre.add("balls must be 1.." + RobotConfig.MAX_BALLS);
                if (!(maxSeconds >= 3 && maxSeconds <= 90) || !(windowMs > 0)) pre.add("invalid duration/window");
                boolean moves = stage >= 3;
                if (moves && (start == null || end == null || Math.hypot(end.x() - start.x(), end.y() - start.y()) < 6))
                    pre.add("path start/end missing or shorter than 6 in");
                Storage.FieldPoint tgt = Storage.activeTarget();
                if (tgt == null || !tgt.isValid()) pre.add("active target point is not set");
                if (!pre.isEmpty()) return ProcedureResult.refused(pre);

                ProcedureResult out = new ProcedureResult();
                RunLog log = new RunLog("t", "x", "y", "heading", "speed", "in_window", "flywheel_ready", "turret_err", "turret_fault",
                        "fly_meas", "fly_target", "phase", "shots", "path_done");
                out.log = log;
                String warn = moves ? "The ROBOT WILL DRIVE from the start pose to the end pose. Area clear, spectators back, operator on BACK to abort. " : "";
                if (!c.confirm(warn + "Stage " + STAGE_NAMES[stage] + (stage >= 2 ? "; balls loaded, shot path clear" : "") + ". A=go")) { out.aborted = true; return out; }

                double v = Storage.activeTargetIndex == 0 ? RobotConfig.FLYWHEEL_VELOCITY_A : RobotConfig.FLYWHEEL_VELOCITY_B;
                boolean oldCfg = TurretConfig.HARDWARE_CONFIGURED;
                double oldVolts = TurretConfig.MAX_CONTROL_VOLTS;
                Command aim = RobotCommands.aimTurret(r), mon = RobotCommands.readinessMonitor(r), main = null;
                WindowTracker tracker = new WindowTracker();
                try {
                    if (moves) r.follower.setPose(start);
                    // honour the committed interlock; only cap the voltage while calibrating
                    TurretConfig.MAX_CONTROL_VOLTS = Math.min(oldVolts, CalConfig.TURRET_MAX_VOLTS * 2);
                    RobotCommands.ensureFlywheelRegulator(r);
                    Scheduler.schedule(aim, mon);
                    Path fwd = moves ? line(start, end).constant(start.heading()) : null;
                    switch (stage) {
                        case 1: main = RobotCommands.spinUpFlywheel(r, v); break;
                        case 2: main = RobotCommands.shoot(r, balls, v, windowMs); break;
                        case 3: main = parallel(follow(r.follower, fwd), RobotCommands.spinUpFlywheel(r, v)); break;
                        case 4: main = RobotCommands.movingShot(r, fwd, 1, v, windowMs); break;
                        case 5: main = RobotCommands.movingShot(r, fwd, balls, v, windowMs); break;
                        default: {
                            Path back = line(end, start).constant(start.heading());
                            main = AutoRoutines.cycle(r, back, fwd, balls, v, windowMs);
                            r.follower.setPose(end);   // the cycle first drives back to the start with the intake running
                        }
                    }
                    Scheduler.schedule(main);
                    double t0 = c.timeSec(), maxSpeed = 0, firstShot = Double.NaN;
                    final int shotBase = Storage.totalBallsShot;
                    int shots = 0, aligned = 0, alignedN = 0;
                    boolean pathDone = false;
                    String lastFail = Storage.lastFailure;
                    while (!c.abortRequested() && c.timeSec() - t0 < maxSeconds) {
                        c.tick();
                        double t = c.timeSec() - t0;
                        ShootingReadiness.Result res = ShootStatus.evaluate(r, !Storage.shootInProgress, "shot in progress");
                        boolean inWindow = res.ready || (Storage.shootInProgress && "shot in progress".equals(res.reason));
                        tracker.sample(t, inWindow, res.reason);
                        Pose p = r.follower.pose(); Velocity vel = r.follower.velocity();
                        double speed = Math.hypot(vel.vx, vel.vy);
                        maxSpeed = Math.max(maxSpeed, speed);
                        if (r.follower.atParametricEnd()) pathDone = true;
                        if (Storage.totalBallsShot - shotBase > shots) { shots = Storage.totalBallsShot - shotBase; if (Double.isNaN(firstShot)) firstShot = t; }
                        boolean fault = Storage.turretFault != null;
                        if (t > 1 && Double.isFinite(Storage.turretError)) { alignedN++; if (!fault && Math.abs(Storage.turretError) <= TurretConfig.ALIGN_TOLERANCE_RAD) aligned++; }
                        log.add(t, p.x(), p.y(), p.heading(), speed, inWindow ? 1 : 0, Storage.flywheelReady ? 1 : 0, Storage.turretError, fault ? 1 : 0,
                                Storage.flywheelMeasuredVelocity, Storage.flywheelTargetVelocity, ShootWindowProcedures.phaseCode(Storage.shootPhase), shots, pathDone ? 1 : 0);
                        if (!Scheduler.isScheduled(main)) break;
                    }
                    out.aborted = c.abortRequested();
                    boolean finished = !Scheduler.isScheduled(main);
                    out.metrics.putAll(tracker.metrics());
                    out.metrics.put("run_time_s", c.timeSec() - t0);
                    out.metrics.put("completed_normally", finished ? 1.0 : 0.0);
                    out.metrics.put("max_speed_in_s", maxSpeed);
                    out.metrics.put("path_completed", pathDone ? 1.0 : 0.0);
                    if (stage >= 2) {
                        int expected = stage == 4 ? 1 : balls;
                        out.metrics.put("shots_ok", (double) shots);
                        out.metrics.put("shot_success_rate", (double) shots / expected);
                        out.metrics.put("first_shot_after_s", firstShot);
                    }
                    out.metrics.put("aligned_fraction", alignedN == 0 ? Double.NaN : (double) aligned / alignedN);
                    double[] te = log.column("turret_err");
                    double mx = 0; for (double e : te) if (Double.isFinite(e)) mx = Math.max(mx, Math.abs(e));
                    out.metrics.put("turret_err_max_deg", Math.toDegrees(mx));
                    for (java.util.Map.Entry<String, Double> e : tracker.blockedReasons().entrySet())
                        out.notes.add(String.format("blocked %.2f s: %s", e.getValue(), e.getKey()));
                    if (Storage.lastFailure != null && !Storage.lastFailure.equals(lastFail)) out.notes.add("failure: " + Storage.lastFailure);
                    if (!finished && !out.aborted) out.notes.add("stage timed out before its commands finished");
                    if (!TurretConfig.HARDWARE_CONFIGURED) out.notes.add("turret interlock is OFF: aiming was computed but the turret was not powered");
                } finally {
                    if (main != null) Scheduler.cancel(main);
                    Scheduler.cancel(aim); Scheduler.cancel(mon);
                    Scheduler.schedule(RobotCommands.stopFlywheel(r));
                    c.tick();
                    r.safeShutdown(false);
                    if (moves) r.follower.hold(r.follower.pose());   // stop driving where we are
                    TurretConfig.MAX_CONTROL_VOLTS = oldVolts;
                    TurretConfig.HARDWARE_CONFIGURED = oldCfg;
                }
                return out;
            }
        };
    }

    @SuppressWarnings("unused") private static double unused() { return Metrics.mean(new double[]{0}); }
}
