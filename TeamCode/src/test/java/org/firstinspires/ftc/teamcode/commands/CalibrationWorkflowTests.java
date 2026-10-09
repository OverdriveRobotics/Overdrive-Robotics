package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.info;
import static org.firstinspires.ftc.teamcode.T.near;
import static org.firstinspires.ftc.teamcode.commands.SimSupport.*;

import org.firstinspires.ftc.teamcode.calibration.CalSession;
import org.firstinspires.ftc.teamcode.calibration.FlywheelProcedures;
import org.firstinspires.ftc.teamcode.calibration.ParamEditor;
import org.firstinspires.ftc.teamcode.calibration.ProcedureResult;
import org.firstinspires.ftc.teamcode.calibration.Procedure;
import org.firstinspires.ftc.teamcode.control.FlywheelConfig;
import org.firstinspires.ftc.teamcode.control.FlywheelLqrController;
import org.firstinspires.ftc.teamcode.control.TurretStateSpaceController;
import org.firstinspires.ftc.teamcode.tuning.ConfigStore;
import org.firstinspires.ftc.teamcode.tuning.ConfigValidator;
import org.firstinspires.ftc.teamcode.tuning.OfflineTuner;
import org.firstinspires.ftc.teamcode.tuning.ParamRegistry;
import org.firstinspires.ftc.teamcode.tuning.RunLog;
import org.firstinspires.ftc.teamcode.tuning.RunReport;
import org.firstinspires.ftc.teamcode.tuning.Tuning;
import org.firstinspires.ftc.teamcode.tuning.cli.ConfigTool;
import org.firstinspires.ftc.teamcode.tuning.cli.LogAnalyzer;
import org.firstinspires.ftc.teamcode.tuning.cli.OfflineTuneCli;
import org.firstinspires.ftc.teamcode.hardware.Storage;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.PrintStream;
import java.nio.file.Files;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * End-to-end calibration workflow in simulation: load -> run -> edit candidate -> rerun -> compare -> save ->
 * accept/reject -> restore, plus the host tools. SIMULATION ONLY.
 */
public final class CalibrationWorkflowTests {
    private CalibrationWorkflowTests() {}

    private static File tmp() throws Exception { return Files.createTempDirectory("overdrive-wf").toFile(); }

    private static String tool(File dir, String... args) throws Exception {
        String[] a = new String[args.length + 1];
        a[0] = dir.getPath();
        System.arraycopy(args, 0, a, 1, args.length);
        ByteArrayOutputStream bo = new ByteArrayOutputStream();
        int rc = ConfigTool.run(a, new PrintStream(bo));
        return rc + "|" + bo.toString();
    }

    public static void run() {
        sim("workflow: baseline -> candidate edit -> rerun -> compare -> save -> reject -> accept -> restore, nothing implicit", s -> {
            File dir = tmp();
            ConfigStore store = new ConfigStore(dir);
            CalSession session = new CalSession(store);
            SimCalContext ctx = new SimCalContext(s);
            Procedure step = FlywheelProcedures.stepResponse(1800, 6);

            RunReport base = session.run(step, ctx, false);
            check(!base.aborted && base.metrics.containsKey("readiness_delay_s"), "baseline ran");
            check("SIMULATION".equals(base.mode) && base.baseRevision == 0 && !base.usedCandidate, "labelled and traceable");
            check(!new File(dir, "active.properties").exists(), "an ordinary run writes NO configuration");

            check(session.setManual("robot.FLYWHEEL_STABLE_MS", 600) == null, "candidate edit accepted");
            check(session.setManual("robot.FLYWHEEL_TOLERANCE", -3) != null, "invalid value refused");
            check(session.setManual("robot.MAX_FLYWHEEL_VELOCITY", 99) != null, "safety limit refused");
            check(session.setMeasured("turret.MAX_CONTROL_VOLTS", 2) != null, "safety cap not settable even as a 'measurement'");
            check(RobotConfig.FLYWHEEL_STABLE_MS == 150, "live config still the active one while only a candidate exists");

            RunReport cand = session.run(step, ctx, true);
            check(cand.usedCandidate && !cand.configHash.equals(base.configHash), "candidate run has its own config hash");
            check(RobotConfig.FLYWHEEL_STABLE_MS == 150, "candidate was applied only for the run and then restored");
            List<String> cmp = session.compareToBaseline("flywheel-step");
            String text = String.join("\n", cmp);
            info("comparison:\n" + text);
            check(text.contains("FLYWHEEL_STABLE_MS") && text.contains("readiness_delay_s"), "parameter and metric deltas shown");
            check(text.matches("(?s).*readiness_delay_s[^\\n]*WORSE.*"), "longer stability window is flagged as worse readiness delay");
            check(cand.metrics.get("readiness_delay_s") > base.metrics.get("readiness_delay_s") + 0.3, "delay increased by roughly the window change");
            File[] runs = store.runsDir().listFiles();
            int csv = 0, rep = 0; for (File f : runs) { if (f.getName().endsWith(".csv")) csv++; if (f.getName().endsWith(".report.properties")) rep++; }
            check(csv == 2 && rep == 2, "two reports and two CSV logs exported");
            File anyCsv = null; for (File f : runs) if (f.getName().endsWith(".csv")) anyCsv = f;
            RunLog log = RunLog.load(anyCsv);
            check(log.meta.get("configHash") != null && log.meta.get("mode").equals("SIMULATION"), "log carries its config hash and mode");

            check(session.saveCandidate("try longer window").isEmpty(), "candidate saved to disk");
            check(new File(dir, "candidate.properties").exists() && !new File(dir, "active.properties").exists(), "saved as candidate, not active");
            session.reject();
            check(!store.hasCandidate() && session.candidateValue("robot.FLYWHEEL_STABLE_MS") == 150, "reject returns to the active settings");

            session.setManual("robot.FLYWHEEL_STABLE_MS", 400);
            check(session.accept("stability 400 ms").isEmpty(), "explicit accept");
            check(session.activeRevision() == 1, "revision 1");
            session.setManual("robot.FLYWHEEL_STABLE_MS", 250);
            check(session.accept("stability 250 ms").isEmpty() && session.activeRevision() == 2, "revision 2");
            check(store.restore(1).isEmpty(), "restore rev 1");
            ParamRegistry.resetToDefaults();
            check(Tuning.applyActive(dir).isEmpty(), "robot boot loads the restored active file");
            near(400, RobotConfig.FLYWHEEL_STABLE_MS, 0, "rev 1 value is live after restore (as new rev 3)");
            check(store.loadActive().revision == 3 && store.revisions().size() == 3, "history preserved, restore is a new revision");
            ParamRegistry.resetToDefaults();
        });

        sim("workflow: baselines persist on disk, so a rerun in a NEW session still compares with the first run", s -> {
            File dir = tmp();
            SimCalContext ctx = new SimCalContext(s);
            CalSession first = new CalSession(new ConfigStore(dir));
            first.run(FlywheelProcedures.stepResponse(1800, 5), ctx, false);
            CalSession second = new CalSession(new ConfigStore(dir));   // e.g. the OpMode was restarted
            second.setManual("robot.FLYWHEEL_STABLE_MS", 500);
            second.run(FlywheelProcedures.stepResponse(1800, 5), ctx, true);
            List<String> cmp = second.compareToBaseline("flywheel-step");
            check(!cmp.isEmpty() && String.join("\n", cmp).contains("readiness_delay_s"), "comparison available after restart: " + cmp);
        });

        sim("workflow: procedures that need target points use ONLY the stored configuration (refuse until they are in it)", s -> {
            CalSession session = new CalSession(new ConfigStore(tmp()));
            SimCalContext ctx = new SimCalContext(s);
            Storage.setTarget(0, 80, 40);   // live memory is NOT configuration
            RunReport r = session.run(() -> org.firstinspires.ftc.teamcode.calibration.TurretProcedures.aimAccuracy(5, 0), ctx, false);
            check(r.aborted && String.join(" ", r.notes).contains("target point is not set"), "refused without configured targets: " + r.notes);
            session.setMeasured("target.A_X", 80); session.setMeasured("target.A_Y", 40);
            RunReport ok = session.run(() -> org.firstinspires.ftc.teamcode.calibration.TurretProcedures.aimAccuracy(5, 0), ctx, true);
            check(!ok.aborted && ok.metrics.containsKey("aim_rms_deg"), "runs once the candidate carries the targets: " + ok.notes);
            check(!Storage.targetA.isValid() || Storage.targetA.x == 80, "live targets restored to what they were before the run");
        });

        sim("workflow: measure -> identify -> adopt -> enable -> rerun under the candidate (LQR path) -> compare", s -> {
            s.noiseStd = 4;
            File dir = tmp();
            CalSession session = new CalSession(new ConfigStore(dir));
            SimCalContext ctx = new SimCalContext(s);
            // 1. SDK-path baseline
            RunReport base = session.run(FlywheelProcedures.stepResponse(1800, 7), ctx, false);
            // 2. identify
            RunReport id = session.run(FlywheelProcedures.sysId(new double[]{0.35, 0.5, 0.65}, 3, 4), ctx, false);
            check(!id.aborted && id.metrics.containsKey("fit_kV"), "identified: " + id.notes);
            check(session.adoptSuggestions().isEmpty(), "suggestions adopted into the CANDIDATE");
            check(session.candidate().containsKey("flywheel.KV_VOLTS_PER_TICK_S") && FlywheelConfig.KV_VOLTS_PER_TICK_S == ParamRegistry.defaultOf("flywheel.KV_VOLTS_PER_TICK_S"),
                    "candidate has the values; live config does not");
            check(session.validateCandidate().isEmpty(), "candidate validates (not yet enabled)");
            // 3. enable the LQR path: allowed only because KV/KA/KS are now explicit
            check(session.setManual("flywheel.MODEL_IDENTIFIED", 1) == null, "interlock request recorded");
            check(session.validateCandidate().isEmpty(), "interlock valid with identified values: " + session.validateCandidate());
            // 4. rerun under the candidate
            RunReport lqr = session.run(FlywheelProcedures.stepResponse(1800, 7), ctx, true);
            check(!lqr.aborted, "ran under candidate: " + lqr.notes);
            check(lqr.notes.contains("control path: LQR"), "LQR path in use: " + lqr.notes);
            info(String.join("\n", RunReport.compare(base, lqr)));
            check(id.metrics.get("fit_r2") > 0.9, "replay r2 of the identification: " + id.metrics.get("fit_r2"));
            check(Math.abs(lqr.metrics.get("steady_state_error")) < 60, "identified LQR holds the target: " + lqr.metrics.get("steady_state_error"));
            check(!FlywheelConfig.MODEL_IDENTIFIED, "live interlock still off until accepted");
            // 5. un-enabled interlock cannot be requested without values
            CalSession fresh = new CalSession(new ConfigStore(tmp()));
            fresh.setManual("flywheel.MODEL_IDENTIFIED", 1);
            RunReport refused = fresh.run(FlywheelProcedures.stepResponse(1800, 5), new SimCalContext(s), true);
            check(refused.aborted && String.join(" ", refused.notes).contains("REFUSED"), "run refused: " + refused.notes);
            check(fresh.accept("x").size() > 0, "accept refused as well");
        });

        sim("workflow: ParamEditor stepping, clamping, toggling and protection", s -> {
            CalSession session = new CalSession(new ConfigStore(tmp()));
            boolean threw = false;
            try { new ParamEditor(session, "turret.MAX_CONTROL_VOLTS"); } catch (IllegalArgumentException e) { threw = true; }
            check(threw, "safety parameter cannot be placed in an editor");
            ParamEditor ed = new ParamEditor(session, "robot.FEED_POWER", "robot.DETECT_INTAKE_JAM", "flywheel.MODEL_STD");
            check(ed.adjust(-1) == null, "step down");
            near(0.95, session.candidateValue("robot.FEED_POWER"), 1e-9, "one step = 0.05");
            for (int i = 0; i < 40; i++) ed.adjust(-1);
            near(0.05, session.candidateValue("robot.FEED_POWER"), 1e-9, "clamped at the minimum");
            ed.move(1);
            ed.adjust(1);
            near(1, session.candidateValue("robot.DETECT_INTAKE_JAM"), 0, "boolean toggles");
            ed.adjust(1);
            near(0, session.candidateValue("robot.DETECT_INTAKE_JAM"), 0, "and back");
            ed.move(1); ed.adjust(3);
            near(ParamRegistry.defaultOf("flywheel.MODEL_STD") + 15, session.candidateValue("flywheel.MODEL_STD"), 1e-9, "3 steps of 5");
            check(ed.describe(0).contains("active"), "shows the active value next to the candidate");
            ed.move(-1); ed.move(-1);
            check(ed.selectedKey().equals("robot.FEED_POWER"), "selection wraps");
        });

        sim("tools: ConfigTool set/diff/accept/restore/known-good on a directory; unsafe edits refused; nothing implicit", s -> {
            File dir = tmp();
            check(tool(dir, "set", "flywheel.MODEL_STD", "88").startsWith("0|"), "set");
            check(tool(dir, "set", "turret.MAX_CONTROL_VOLTS", "2").startsWith("1|"), "safety parameter refused");
            check(tool(dir, "set", "robot.FLYWHEEL_TOLERANCE", "-4").startsWith("1|"), "invalid value refused");
            check(tool(dir, "set", "no.such", "1").startsWith("1|"), "unknown key refused");
            check(tool(dir, "diff").contains("flywheel.MODEL_STD"), "diff shows the change");
            check(tool(dir, "show").contains("no overrides"), "active still defaults");
            check(tool(dir, "accept", "first").startsWith("0|") && tool(dir, "show").contains("revision 1"), "accept");
            tool(dir, "set", "flywheel.MODEL_STD", "99"); tool(dir, "accept", "second");
            tool(dir, "good", "1");
            check(tool(dir, "history").contains("[known-good]"), "history marks known-good");
            check(tool(dir, "restore-good").contains("restored latest known-good"), "restore known-good");
            check(tool(dir, "show").contains("flywheel.MODEL_STD = 88.0") && tool(dir, "show").contains("revision 3"), "values of rev 1 are active");
            tool(dir, "set", "flywheel.MODEL_STD", "55");
            check(tool(dir, "reject").contains("rejected") && tool(dir, "candidate").contains("no candidate"), "reject");
            check(tool(dir, "params").contains("PLACEHOLDER".toLowerCase()) || tool(dir, "params").contains("placeholder"), "params listing flags placeholders");
        });

        sim("tools: OfflineTuner ranks weights in simulation, never beats reality claims, and writes only a candidate", s -> {
            FlywheelLqrController.Params nominal = FlywheelLqrController.Params.physical();
            OfflineTuner.Result r = OfflineTuner.tuneFlywheel(nominal, 1800, 1);
            info("flywheel: default score " + r.defaultScore + " best " + r.bestScore + " at " + r.best);
            check(r.bestScore <= r.defaultScore + 1e-9, "best never worse than the default point");
            check(ConfigValidator.validate(new TreeMap<>(r.best)).isEmpty(), "result validates");
            check(r.disclaimer.startsWith("SIMULATION"), "labelled");
            OfflineTuner.Result t = OfflineTuner.tuneTurret(TurretStateSpaceController.Params.fromConfig(), 1);
            info("turret: default score " + t.defaultScore + " best " + t.bestScore + " at " + t.best);
            check(t.bestScore <= t.defaultScore + 1e-9 && ConfigValidator.validate(new TreeMap<>(t.best)).isEmpty(), "turret result sane");
            check(OfflineTuner.scoreFlywheel(nominal, 100, 50, 1800, 1) == OfflineTuner.scoreFlywheel(nominal, 100, 50, 1800, 1), "deterministic for a seed");
            File dir = tmp();
            ByteArrayOutputStream bo = new ByteArrayOutputStream();
            check(OfflineTuneCli.run(new String[]{dir.getPath(), "flywheel", "1800"}, new PrintStream(bo)) == 0, "cli ran: " + bo);
            check(new File(dir, "candidate.properties").exists() && !new File(dir, "active.properties").exists(), "candidate only");
            check(bo.toString().contains("SIMULATION"), "output labelled");
        });

        sim("tools: LogAnalyzer reproduces the on-robot sysid fit from the exported CSV", s -> {
            s.noiseStd = 3;
            File dir = tmp();
            CalSession session = new CalSession(new ConfigStore(dir));
            RunReport id = session.run(FlywheelProcedures.sysId(new double[]{0.35, 0.5, 0.65}, 3, 4), new SimCalContext(s), false);
            File csv = null;
            for (File f : session.store().runsDir().listFiles()) if (f.getName().endsWith(".csv")) csv = f;
            ByteArrayOutputStream bo = new ByteArrayOutputStream();
            check(LogAnalyzer.run(new String[]{"sysid", csv.getPath()}, new PrintStream(bo)) == 0, "analyzer ran: " + bo);
            String out = bo.toString();
            check(out.contains("kV=") && out.contains("configHash"), "prints the fit and the traceability metadata: " + out);
            double kv = Double.parseDouble(out.replaceAll("(?s).*kV=([0-9.eE-]+).*", "$1"));
            near(id.metrics.get("fit_kV"), kv, id.metrics.get("fit_kV") * 1e-3, "same answer offline");
        });

        sim("tools: RunReport files can be compared from the command line", s -> {
            File dir = tmp();
            CalSession session = new CalSession(new ConfigStore(dir));
            SimCalContext ctx = new SimCalContext(s);
            session.run(FlywheelProcedures.stepResponse(1800, 5), ctx, false);
            session.setManual("robot.FLYWHEEL_TOLERANCE", 100);
            session.run(FlywheelProcedures.stepResponse(1800, 5), ctx, true);
            File[] reps = session.store().runsDir().listFiles((d, n) -> n.endsWith(".report.properties"));
            java.util.Arrays.sort(reps);
            String out = tool(dir, "compare", reps[0].getPath(), reps[1].getPath());
            check(out.startsWith("0|") && out.contains("settling_time_s") && out.contains("FLYWHEEL_TOLERANCE"), out);
        });
    }
}
