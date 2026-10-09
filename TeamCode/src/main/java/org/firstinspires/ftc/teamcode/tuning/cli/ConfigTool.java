package org.firstinspires.ftc.teamcode.tuning.cli;

import org.firstinspires.ftc.teamcode.tuning.ConfigStore;
import org.firstinspires.ftc.teamcode.tuning.ParamRegistry;
import org.firstinspires.ftc.teamcode.tuning.Param;
import org.firstinspires.ftc.teamcode.tuning.RunReport;

import java.io.File;
import java.util.List;
import java.util.Map;

/**
 * Host-side configuration manager for a directory pulled from (or pushed to) the robot controller.
 * Usage: ConfigTool DIR (show | candidate | diff | set KEY VALUE | unset KEY | accept [NOTE] | reject |
 *                        history | restore REV | good REV | restore-good | params | compare REPORT_A REPORT_B)
 * Every mutating action is explicit; nothing is changed by running tests or procedures.
 */
public final class ConfigTool {
    private ConfigTool() {}

    public static void main(String[] a) throws Exception {
        System.exit(run(a, System.out));
    }

    /** Markdown reference of every registered parameter (docs/PARAMETERS.md is generated from this and checked by a test). */
    public static String parameterTable() {
        StringBuilder sb = new StringBuilder();
        sb.append("| Key | Category | Default | Range | Unit | Hand-adjustable | Description |\n|---|---|---|---|---|---|---|\n");
        for (Param p : ParamRegistry.all().values()) {
            double d = ParamRegistry.defaultOf(p.key);
            String def = Double.isNaN(d) ? "unset" : p.format(d);
            sb.append("| `").append(p.key).append("` | ").append(p.category).append(p.placeholder ? " (placeholder default)" : "").append(" | ").append(def)
                    .append(" | ").append(p.format(p.min)).append(" .. ").append(p.format(p.max)).append(" | ").append(p.unit).append(" | ")
                    .append(p.manuallyAdjustable() ? "yes" : (p.category == Param.Category.SAFETY ? (p.tighten == Param.Tighten.NONE ? "measured only" : "tighten-only (file)") : "no"))
                    .append(" | ").append(p.description.replace("|", "/")).append(" |\n");
        }
        return sb.toString();
    }

    public static int run(String[] a, java.io.PrintStream out) throws Exception {
        if (a.length < 2) { out.println("usage: ConfigTool DIR (show|candidate|diff|set K V|set-measured K V|unset K|accept [NOTE]|reject|history|restore REV|good REV|restore-good|params|params-md|compare A B)"); return 2; }
        ConfigStore st = new ConfigStore(new File(a[0]));
        String cmd = a[1];
        switch (cmd) {
            case "show": {
                ConfigStore.Config c = st.loadActive();
                out.println("active revision " + c.revision + " (parent " + c.parent + ") cfg " + c.hash() + "  note: " + c.note);
                for (Map.Entry<String, Double> e : c.overrides.entrySet()) out.println("  " + e.getKey() + " = " + e.getValue());
                if (c.overrides.isEmpty()) out.println("  (no overrides: compiled defaults)");
                return 0;
            }
            case "candidate": {
                if (!st.hasCandidate()) { out.println("no candidate"); return 0; }
                ConfigStore.Config c = st.loadCandidate();
                out.println("candidate based on revision " + c.parent + ":");
                for (Map.Entry<String, Double> e : c.overrides.entrySet()) out.println("  " + e.getKey() + " = " + e.getValue());
                return 0;
            }
            case "diff": {
                Map<String, Double> cur = st.candidateOrActive();
                List<String> d = ConfigStore.diff(st.loadActive().overrides, cur);
                if (!st.hasCandidate()) out.println("(no candidate: showing active vs active)");
                for (String s : d) out.println("  " + s);
                if (d.isEmpty()) out.println("  no differences");
                return 0;
            }
            case "set": {
                if (a.length != 4) { out.println("set KEY VALUE"); return 2; }
                Param p = ParamRegistry.get(a[2]);
                if (p == null) { out.println("unknown parameter " + a[2]); return 1; }
                if (!p.manuallyAdjustable()) { out.println(a[2] + " is " + p.category + ": not adjustable with this tool (edit source deliberately)"); return 1; }
                Map<String, Double> c = st.candidateOrActive();
                c.put(a[2], Double.parseDouble(a[3]));
                List<String> errs = st.saveCandidate(c, "set " + a[2]);
                for (String e : errs) out.println("REFUSED: " + e);
                if (errs.isEmpty()) out.println("candidate updated (not active). Review with 'diff', then 'accept' or 'reject'.");
                return errs.isEmpty() ? 0 : 1;
            }
            case "set-measured": {   // measured-only values (e.g. turret hard-stop limits) that "set" refuses; use ONLY numbers you measured
                if (a.length != 4) { out.println("set-measured KEY VALUE"); return 2; }
                Param p = ParamRegistry.get(a[2]);
                if (p == null || p.category != Param.Category.SAFETY || p.tighten != Param.Tighten.NONE) { out.println(a[2] + " is not a measured-only value; use 'set'"); return 1; }
                Map<String, Double> c = st.candidateOrActive();
                c.put(a[2], Double.parseDouble(a[3]));
                List<String> errs = st.saveCandidate(c, "measured " + a[2]);
                for (String e : errs) out.println("REFUSED: " + e);
                if (errs.isEmpty()) out.println("candidate updated (not active)");
                return errs.isEmpty() ? 0 : 1;
            }
            case "unset": {
                Map<String, Double> c = st.candidateOrActive();
                c.remove(a[2]);
                List<String> errs = st.saveCandidate(c, "unset " + a[2]);
                for (String e : errs) out.println("REFUSED: " + e);
                return errs.isEmpty() ? 0 : 1;
            }
            case "accept": {
                List<String> errs = st.accept(a.length > 2 ? a[2] : null);
                for (String e : errs) out.println("REFUSED: " + e);
                if (errs.isEmpty()) out.println("ACCEPTED as revision " + st.loadActive().revision + ". Re-run the automated tests, then push the directory to the robot.");
                return errs.isEmpty() ? 0 : 1;
            }
            case "reject": out.println(st.reject() ? "candidate rejected (copy kept in rejected/)" : "no candidate"); return 0;
            case "history": {
                List<Integer> kg = st.knownGood();
                for (int r : st.revisions()) {
                    ConfigStore.Config c = st.loadRevision(r);
                    out.println("  rev " + r + (kg.contains(r) ? " [known-good]" : "") + "  parent " + c.parent + "  cfg " + c.hash() + "  " + c.note);
                }
                return 0;
            }
            case "restore": {
                List<String> errs = st.restore(Integer.parseInt(a[2]));
                for (String e : errs) out.println("REFUSED: " + e);
                if (errs.isEmpty()) out.println("restored as revision " + st.loadActive().revision);
                return errs.isEmpty() ? 0 : 1;
            }
            case "good": st.markKnownGood(Integer.parseInt(a[2])); out.println("marked rev " + a[2] + " known-good"); return 0;
            case "restore-good": {
                List<String> errs = st.restoreLatestKnownGood();
                for (String e : errs) out.println("REFUSED: " + e);
                if (errs.isEmpty()) out.println("restored latest known-good as revision " + st.loadActive().revision);
                return errs.isEmpty() ? 0 : 1;
            }
            case "params": {
                for (Param p : ParamRegistry.all().values())
                    out.println(String.format("  %-44s %-14s [%s..%s] %s%s", p.key, p.category, p.min, p.max, p.unit, p.placeholder ? "  (placeholder default)" : ""));
                return 0;
            }
            case "params-md": out.print(parameterTable()); return 0;
            case "compare": {
                for (String s : RunReport.compare(RunReport.load(new File(a[2])), RunReport.load(new File(a[3])))) out.println(s);
                return 0;
            }
            default: out.println("unknown command " + cmd); return 2;
        }
    }
}
