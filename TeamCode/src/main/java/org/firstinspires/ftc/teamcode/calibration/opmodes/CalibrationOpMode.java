package org.firstinspires.ftc.teamcode.calibration.opmodes;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.calibration.CalContext;
import org.firstinspires.ftc.teamcode.calibration.CalSession;
import org.firstinspires.ftc.teamcode.calibration.ParamEditor;
import org.firstinspires.ftc.teamcode.calibration.Procedure;
import org.firstinspires.ftc.teamcode.hardware.RobotClock;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Storage;
import org.firstinspires.ftc.teamcode.tuning.ConfigStore;
import org.firstinspires.ftc.teamcode.tuning.ParamRegistry;
import org.firstinspires.ftc.teamcode.tuning.RunReport;
import org.firstinspires.ftc.teamcode.tuning.Tuning;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

/**
 * Shared guided-calibration OpMode. HARDWARE ONLY: needs the assembled robot. Subclasses list their procedures and the
 * parameters that can be tuned; everything else (menu, safety, configuration workflow) lives here.
 *
 * Controls (gamepad 1):
 *   BACK ........ ABORT the running procedure at any time (and answers every prompt with "no")
 *   START ....... switch between the PROCEDURES and PARAMETERS panes
 *   D-pad up/down select;  PARAMETERS pane: D-pad left/right = -/+ one step, bumpers = -/+ ten steps
 *   PROCEDURES: A = run with the CANDIDATE settings, X = run with the ACTIVE settings, Y = adopt this run's suggestions into the candidate
 *   RIGHT STICK (press) opens the configuration menu: save / accept / reject candidate, restore known-good, mark known-good
 * Nothing is saved or made active without an explicit choice in the configuration menu AND a confirmation.
 * The OpMode always leaves the robot in the safe state (Scheduler.reset() + RobotHardware.safeShutdown) on exit.
 */
public abstract class CalibrationOpMode extends LinearOpMode implements CalContext {

    /** Menu entry: procedures are built on demand so their arguments follow the configuration they run under. */
    protected static final class Item {
        final String name; final Supplier<Procedure> factory;
        public Item(String name, Supplier<Procedure> factory) { this.name = name; this.factory = factory; }
    }

    protected final RobotHardware robot = new RobotHardware();
    protected CalSession session;
    protected ParamEditor editor;
    private final List<String> messages = new ArrayList<>();
    private String statusLine = "";
    private boolean paramPane;
    private int procSel;
    private RunReport last;
    private Map<String, Double> lastSuggestions = new java.util.LinkedHashMap<>();
    private List<String> lastCompare = new ArrayList<>();
    private long lastTelemetryNanos;
    private boolean abortFlag;
    private final boolean[] prev = new boolean[16];

    protected abstract String title();
    protected abstract List<Item> items();
    /** Registry keys the operator may tune in this OpMode (must all be manually adjustable). */
    protected abstract String[] tunableKeys();

    // ------------------------------------------------------------------ lifecycle

    @Override public final void runOpMode() throws InterruptedException {
        try {
            robot.init(hardwareMap, 0, 0, 0);   // also loads the ACTIVE configuration
            Scheduler.reset();
            ConfigStore store = new ConfigStore(Tuning.robotDir(AppUtil.FIRST_FOLDER));
            session = new CalSession(store);
            editor = tunableKeys().length > 0 ? new ParamEditor(session, tunableKeys()) : null;
            telemetry.addLine(title() + "  [HARDWARE]");
            telemetry.addLine("Motors will not move until you start a procedure and confirm its prompt.");
            telemetry.addLine("BACK = abort at any time. Press START on the driver station to begin.");
            telemetry.update();
            waitForStart();
            menuLoop();
        } catch (java.io.IOException e) {
            telemetry.addLine("configuration error: " + e.getMessage());
            telemetry.update();
            while (opModeIsActive()) sleep(50);
        } finally {
            // Scheduler.reset() does NOT stop hardware; the explicit shutdown does.
            Scheduler.reset();
            robot.safeShutdown(true);
        }
    }

    // ------------------------------------------------------------------ menu

    private void menuLoop() throws java.io.IOException {
        while (opModeIsActive()) {
            List<Item> items = items();
            if (pressed(0, gamepad1.start)) paramPane = !paramPane && editor != null;
            if (pressed(1, gamepad1.dpad_down)) { if (paramPane) editor.move(1); else procSel = Math.min(items.size() - 1, procSel + 1); }
            if (pressed(2, gamepad1.dpad_up)) { if (paramPane) editor.move(-1); else procSel = Math.max(0, procSel - 1); }
            if (paramPane) {
                int steps = 0;
                if (pressed(3, gamepad1.dpad_right)) steps = 1;
                if (pressed(4, gamepad1.dpad_left)) steps = -1;
                if (pressed(5, gamepad1.right_bumper)) steps = 10;
                if (pressed(6, gamepad1.left_bumper)) steps = -10;
                if (steps != 0) { String err = editor.adjust(steps); if (err != null) message(err); }
            } else if (!items.isEmpty()) {
                if (pressed(7, gamepad1.a)) run(items.get(procSel), true);
                if (pressed(8, gamepad1.x)) run(items.get(procSel), false);
                if (pressed(9, gamepad1.y)) {
                    List<String> problems = session.adoptSuggestions();
                    message(problems.isEmpty() ? "suggestions copied into the CANDIDATE (not saved, not active)" : "some suggestions rejected: " + problems);
                }
            }
            if (pressed(10, gamepad1.right_stick_button)) configMenu();
            render();
            sleep(20);
        }
    }

    private void run(Item item, boolean useCandidate) {
        abortFlag = false;
        message("running " + item.name + " with " + (useCandidate ? "CANDIDATE" : "ACTIVE") + " settings");
        try {
            last = session.run(item.factory, this, useCandidate);
            lastSuggestions = session.suggestions();
            lastCompare = session.compareToBaseline(last.procedure);
            message(item.name + (last.aborted ? " ABORTED/REFUSED" : " finished") + ": see results");
        } catch (Exception e) {
            message("procedure failed: " + e);
        } finally {
            // Whatever happened, mechanisms return to their defined safe state (flywheel left alone only if a procedure wants it; stopped here).
            Scheduler.reset();
            robot.safeShutdown(true);
        }
    }

    private void configMenu() throws java.io.IOException {
        String[] opts = {"Save candidate to file", "ACCEPT candidate (make it active)", "REJECT candidate", "Restore latest known-good", "Mark ACTIVE revision known-good", "Close"};
        int sel = 0;
        while (opModeIsActive()) {
            if (pressed(11, gamepad1.dpad_down)) sel = Math.min(opts.length - 1, sel + 1);
            if (pressed(12, gamepad1.dpad_up)) sel = Math.max(0, sel - 1);
            if (pressed(13, gamepad1.b)) return;
            if (pressed(14, gamepad1.a)) {
                switch (sel) {
                    case 0: { List<String> e = session.saveCandidate("saved from " + title()); message(e.isEmpty() ? "candidate saved (not active)" : "REFUSED: " + e); break; }
                    case 1: if (ask("ACCEPT: the candidate becomes revision " + (session.activeRevision() + 1) + ". Continue?")) {
                        List<String> e = session.accept("accepted from " + title());
                        message(e.isEmpty() ? "ACCEPTED as revision " + session.activeRevision() + ". Re-run the automated tests on the PC, then verify on the robot." : "REFUSED: " + e);
                    } break;
                    case 2: if (ask("REJECT the candidate and return to the active settings?")) { session.reject(); message("candidate rejected"); } break;
                    case 3: if (ask("Restore the latest known-good revision as the new active configuration?")) {
                        List<String> e = session.store().restoreLatestKnownGood(); session.reloadFromDisk();
                        message(e.isEmpty() ? "restored: active revision " + session.activeRevision() : "REFUSED: " + e);
                    } break;
                    case 4: if (ask("Mark active revision " + session.activeRevision() + " as verified known-good?")) { session.store().markKnownGood(session.activeRevision()); message("marked known-good"); } break;
                    default: return;
                }
            }
            telemetry.addLine("CONFIGURATION MENU  (A select, B close)");
            for (int i = 0; i < opts.length; i++) telemetry.addLine((i == sel ? "> " : "  ") + opts[i]);
            telemetry.addData("active revision", session.activeRevision());
            telemetry.addData("candidate changes", session.candidateDiff().size());
            for (String d : session.candidateDiff()) telemetry.addLine("  " + d);
            if (!messages.isEmpty()) telemetry.addLine(messages.get(messages.size() - 1));
            telemetry.update();
            sleep(20);
        }
    }

    private boolean ask(String q) { return confirm(q); }

    private void render() {
        telemetry.addLine(title() + "  [HARDWARE]   battery " + String.format("%.2f", robot.batteryVolts()) + " V   active rev " + session.activeRevision());
        telemetry.addLine("pane: " + (paramPane ? "PARAMETERS" : "PROCEDURES") + "  (START switches)   BACK=abort   R-stick=config menu");
        List<Item> items = items();
        telemetry.addLine("--- procedures (A=run candidate, X=run active, Y=adopt suggestions) ---");
        for (int i = 0; i < items.size(); i++) telemetry.addLine((!paramPane && i == procSel ? "> " : "  ") + items.get(i).name);
        if (editor != null) {
            telemetry.addLine("--- parameters (candidate; D-pad L/R, bumpers x10) ---");
            for (int i = 0; i < editor.keys().length; i++) telemetry.addLine((paramPane ? "" : "  ") + editor.describe(i));
        }
        List<String> diff = session.candidateDiff();
        telemetry.addData("candidate vs active", diff.isEmpty() ? "no differences" : diff.size() + " change(s)");
        if (last != null) {
            telemetry.addLine("--- last run: " + last.procedure + " [" + last.mode + "] " + (last.aborted ? "ABORTED/REFUSED" : "ok") + (last.usedCandidate ? " (candidate)" : " (active)") + " cfg " + last.configHash);
            int n = 0;
            for (Map.Entry<String, Double> e : last.metrics.entrySet()) { if (n++ >= 12) break; telemetry.addData(e.getKey(), String.format("%.4g", e.getValue())); }
            for (String s : last.notes) telemetry.addLine("note: " + s);
            for (String s : lastCompare) telemetry.addLine(s);
            if (!lastSuggestions.isEmpty()) telemetry.addData("suggestions (Y adopts)", lastSuggestions.toString());
        }
        for (int i = Math.max(0, messages.size() - 3); i < messages.size(); i++) telemetry.addLine("» " + messages.get(i));
        telemetry.update();
    }

    private void message(String m) { messages.add(m); }

    private boolean pressed(int idx, boolean now) { boolean r = now && !prev[idx]; prev[idx] = now; return r; }

    // ------------------------------------------------------------------ CalContext (used by procedures)

    @Override public RobotHardware robot() { return robot; }
    @Override public String mode() { return "HARDWARE"; }

    @Override public void tick() {
        robot.updateLocalization();
        Scheduler.execute();
        long now = RobotClock.nanos();
        if (now - lastTelemetryNanos > 150_000_000L) {
            lastTelemetryNanos = now;
            telemetry.addLine("PROCEDURE RUNNING  (BACK = ABORT)");
            telemetry.addData("status", statusLine);
            telemetry.addData("battery V", String.format("%.2f", robot.batteryVolts()));
            telemetry.addData("flywheel tgt/meas", String.format("%.0f / %.0f", Storage.flywheelTargetVelocity, Storage.flywheelMeasuredVelocity));
            telemetry.addData("turret deg", Double.isNaN(Storage.turretAngle) ? "n/a" : String.format("%.1f", Math.toDegrees(Storage.turretAngle)));
            telemetry.update();
        }
        if (gamepad1.back || gamepad2.back) abortFlag = true;
        idle();
    }

    @Override public double timeSec() { return RobotClock.nanos() / 1e9; }
    @Override public boolean abortRequested() { return abortFlag || isStopRequested() || gamepad1.back || gamepad2.back; }

    @Override public boolean confirm(String prompt) {
        statusLine = prompt;
        boolean pa = true, pb = true;   // swallow a button already held when the prompt opens
        while (!abortRequested()) {
            telemetry.addLine("OPERATOR PROMPT");
            telemetry.addLine(prompt);
            telemetry.addLine("A = yes     B = no     BACK = abort");
            telemetry.update();
            robot.updateLocalization();
            Scheduler.execute();
            boolean a = gamepad1.a, b = gamepad1.b;
            if (!a) pa = false;
            if (!b) pb = false;
            if (a && !pa) { sleepWhileReleased(); return true; }
            if (b && !pb) { sleepWhileReleased(); return false; }
            idle();
        }
        return false;
    }

    private void sleepWhileReleased() { while (opModeIsActive() && (gamepad1.a || gamepad1.b)) idle(); }

    @Override public double promptValue(String prompt, double initial, double step, double min, double max) {
        double v = initial;
        boolean up = true, down = true, a = true;
        while (!abortRequested()) {
            telemetry.addLine("ENTER VALUE");
            telemetry.addLine(prompt);
            telemetry.addData("value", v);
            telemetry.addLine("D-pad up/down = +/- " + step + "   bumpers = +/- " + 10 * step + "   A = accept   BACK = abort");
            telemetry.update();
            robot.updateLocalization();
            Scheduler.execute();
            if (gamepad1.dpad_up && !up) v += step;
            if (gamepad1.dpad_down && !down) v -= step;
            if (gamepad1.right_bumper && !prev[15]) v += 10 * step;
            if (gamepad1.left_bumper && !prev[14]) v -= 10 * step;
            prev[15] = gamepad1.right_bumper; prev[14] = gamepad1.left_bumper;
            up = gamepad1.dpad_up; down = gamepad1.dpad_down;
            v = Math.max(min, Math.min(max, v));
            if (gamepad1.a && !a) { while (opModeIsActive() && gamepad1.a) idle(); return v; }
            a = gamepad1.a;
            idle();
        }
        return initial;
    }

    @Override public void status(String line) { statusLine = line; }

    @SuppressWarnings("unused") private void unused() { ParamRegistry.all(); }
}
