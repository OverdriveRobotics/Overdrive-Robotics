package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.calibration.*;
import org.firstinspires.ftc.teamcode.control.FlywheelConfig;
import org.firstinspires.ftc.teamcode.control.TurretConfig;
import org.firstinspires.ftc.teamcode.hardware.Storage;
import org.firstinspires.ftc.teamcode.tuning.ConfigStore;
import org.firstinspires.ftc.teamcode.tuning.RunReport;

import java.io.File;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

/**
 * Runs a calibration procedure against the SIMULATOR (never hardware), through the same CalSession workflow the robot
 * uses: reports + CSV logs go to DIR/runs, suggestions can be adopted into DIR/candidate.properties.
 *
 *   SimCalibrate DIR PROCEDURE [--candidate] [--adopt] [--sim-targets] [--set KEY=VALUE ...]
 *   (turret-aim, window*, stage* need target points in the CONFIGURATION: use --sim-targets, or set target.A_X/A_Y first)
 *   PROCEDURE: flywheel-step | flywheel-sysid | flywheel-disturbance | flywheel-scaling | turret-direction | turret-ticksperrev |
 *              turret-limits | turret-step | turret-aim | intake | stopper-timing | stopper-endpoints | window | window-shoot |
 *              stage1 .. stage6 | loc-velframe
 * Every result is labelled SIMULATION: it exercises the tooling and the controllers, it does not validate the robot.
 */
public final class SimCalibrate {
    private SimCalibrate() {}

    public static void main(String[] a) throws Exception { System.exit(run(a, System.out)); }

    public static int run(String[] a, java.io.PrintStream out) throws Exception {
        if (a.length < 2) { out.println("usage: SimCalibrate DIR PROCEDURE [--candidate] [--adopt] [--sim-targets] [--set KEY=VALUE ...]"); return 2; }
        File dir = new File(a[0]);
        String name = a[1];
        boolean useCandidate = false, adopt = false;
        ConfigStore store = new ConfigStore(dir);
        CalSession session = new CalSession(store);
        for (int i = 2; i < a.length; i++) {
            if (a[i].equals("--candidate")) useCandidate = true;
            else if (a[i].equals("--adopt")) adopt = true;
            else if (a[i].equals("--sim-targets")) {   // SIMULATION convenience: invented target points, written to the CANDIDATE only
                session.setMeasured("target.A_X", 80); session.setMeasured("target.A_Y", 40);
                session.setMeasured("target.B_X", 80); session.setMeasured("target.B_Y", -40);
                useCandidate = true;
                out.println("note: --sim-targets put INVENTED target points in the candidate; procedures run under the candidate");
            }
            else if (a[i].equals("--set")) {
                String[] kv = a[++i].split("=");
                String err = session.setManual(kv[0], Double.parseDouble(kv[1]));
                if (err != null) { out.println("REFUSED: " + err); return 1; }
                useCandidate = true;
            }
        }
        SimRobot s = SimRobot.fresh();
        try {
            s.noiseStd = 4;
            s.follower.pose = new Pose(10, 10, 0);
            s.follower.followDurationLoops = 300;
            s.ballsLoaded = 10;
            SimCalContext c = new SimCalContext(s);
            c.echo = true;
            c.onLoop = x -> { x.follower.vel = new Velocity(13, 0, 0); };
            Supplier<Procedure> p = make(name, s, c);
            if (p == null) { out.println("unknown procedure " + name); return 2; }
            out.println("=== SIMULATION run: " + name + (useCandidate ? " (candidate settings)" : " (active settings)") + " ===");
            RunReport r = session.run(p, c, useCandidate);
            out.println("config hash " + r.configHash + ", base revision " + r.baseRevision + (r.aborted ? "   [ABORTED/REFUSED]" : ""));
            for (Map.Entry<String, Double> e : r.metrics.entrySet()) out.println(String.format("  %-30s %s", e.getKey(), e.getValue()));
            for (String n : r.notes) out.println("  note: " + n);
            List<String> cmp = session.compareToBaseline(r.procedure);
            if (!cmp.isEmpty()) { out.println("--- comparison with the first run of this procedure ---"); for (String l : cmp) out.println(l); }
            Map<String, Double> sug = session.suggestions();
            if (!sug.isEmpty()) {
                out.println("suggestions: " + sug);
                if (adopt) {
                    List<String> problems = session.adoptSuggestions();
                    List<String> errs = session.saveCandidate("adopted from simulated " + name);
                    out.println(problems.isEmpty() && errs.isEmpty() ? "adopted into CANDIDATE (not active): review with ConfigTool DIR diff"
                            : "partially adopted; problems: " + problems + errs);
                }
            }
            out.println("report + log written under " + store.runsDir());
            return 0;
        } finally {
            s.cleanup();
        }
    }

    private static Supplier<Procedure> make(String name, final SimRobot s, final SimCalContext c) {
        switch (name) {
            case "flywheel-step": return () -> FlywheelProcedures.stepResponse(RobotConfig.FLYWHEEL_VELOCITY_A, 6);
            case "flywheel-sysid": return () -> FlywheelProcedures.sysId(new double[]{0.35, 0.5, 0.65}, 3, 4);
            case "flywheel-disturbance": return () -> FlywheelProcedures.disturbance(RobotConfig.FLYWHEEL_VELOCITY_A, 200);
            case "flywheel-scaling":
                c.operator = new SimCalContext.Operator() {
                    @Override public boolean confirm(String p, SimRobot x) { return true; }
                    @Override public double value(String p, double i, SimRobot x) { return x.omega / (FlywheelConfig.SHAFT_TICKS_PER_REV * 1.1) * 60; }
                };
                return () -> FlywheelProcedures.scaling(0.3);
            case "turret-direction": return TurretProcedures::encoderDirection;
            case "turret-ticksperrev":
                c.operator = new SimCalContext.Operator() {
                    @Override public boolean confirm(String p, SimRobot x) {
                        if (p.contains("mark 0")) { x.theta = 0; x.step(10); }
                        if (p.contains("Rotate it BY HAND")) { x.theta = Math.toRadians(90) * 1.01; x.step(10); }
                        return true;
                    }
                    @Override public double value(String p, double i, SimRobot x) { return i; }
                };
                return () -> TurretProcedures.ticksPerRev(3, 90);
            case "turret-limits":
                c.operator = new SimCalContext.Operator() {
                    @Override public boolean confirm(String p, SimRobot x) {
                        if (p.contains("MINIMUM")) { x.theta = -1.7; x.step(10); }
                        if (p.contains("MAXIMUM")) { x.theta = 1.6; x.step(10); }
                        return true;
                    }
                    @Override public double value(String p, double i, SimRobot x) { return i; }
                };
                return TurretProcedures::limits;
            case "turret-step": return () -> TurretProcedures.stepResponse(20, 2.5);
            case "turret-aim": return () -> TurretProcedures.aimAccuracy(10, 5);
            case "intake": s.intake.currentAmps = 3.0; return () -> IntakeProcedures.directionAndTransfer(3, 800);
            case "stopper-timing":
                c.operator = new SimCalContext.Operator() {
                    @Override public boolean confirm(String p, SimRobot x) { if (p.contains("settled")) x.run(150); return true; }
                    @Override public double value(String p, double i, SimRobot x) { return i; }
                };
                return () -> StopperProcedures.timing(4);
            case "stopper-endpoints": return StopperProcedures::endpoints;
            case "window": return () -> ShootWindowProcedures.monitor(10, 0);
            case "window-shoot": return () -> ShootWindowProcedures.monitor(25, 3);
            case "loc-velframe":
                final double[] k = {0};
                c.onLoop = x -> {
                    k[0] += 0.01;
                    double h = Math.toRadians(90), fx = 18 * Math.cos(1.5 * k[0]), fy = 18 * Math.sin(1.5 * k[0]);
                    Pose cur = x.follower.pose;
                    x.follower.pose = new Pose(cur.x() + fx * 0.01, cur.y() + fy * 0.01, h);
                    x.follower.vel = new Velocity(fx, fy, 0);
                };
                return () -> LocalizationProcedures.velocityFrame(10);
            default:
                if (name.matches("stage[1-6]")) {
                    final int st = name.charAt(5) - '0';
                    c.onLoop = x -> { x.follower.vel = new Velocity(13, 0, 0); x.follower.pose = new Pose(10 + Math.min(40, x.follower.loopsSinceFollow * 0.13), 10, 0); };
                    return () -> MovingShotProcedures.stage(st, new Pose(10, 10, 0), new Pose(50, 10, 0), 3, 3000, 40);
                }
                return null;
        }
    }

    @SuppressWarnings("unused") private static void unused() { TurretConfig.class.getName(); }
}
