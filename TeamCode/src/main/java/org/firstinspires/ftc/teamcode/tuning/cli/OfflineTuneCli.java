package org.firstinspires.ftc.teamcode.tuning.cli;

import org.firstinspires.ftc.teamcode.control.FlywheelLqrController;
import org.firstinspires.ftc.teamcode.control.TurretStateSpaceController;
import org.firstinspires.ftc.teamcode.tuning.ConfigStore;
import org.firstinspires.ftc.teamcode.tuning.OfflineTuner;
import org.firstinspires.ftc.teamcode.tuning.ParamRegistry;

import java.io.File;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * OfflineTuneCli DIR (flywheel TARGET | turret): loads the ACTIVE configuration from DIR (so the simulated plant uses
 * your identified values), searches controller weights in simulation, prints the ranking and writes the best set as a
 * CANDIDATE (never active). The candidate must still be reviewed, run on the robot and accepted.
 */
public final class OfflineTuneCli {
    private OfflineTuneCli() {}

    public static void main(String[] a) throws Exception { System.exit(run(a, System.out)); }

    public static int run(String[] a, java.io.PrintStream out) throws Exception {
        if (a.length < 2) { out.println("usage: OfflineTuneCli DIR (flywheel TARGET_TICKS_PER_S | turret)"); return 2; }
        ConfigStore st = new ConfigStore(new File(a[0]));
        Map<String, Double> active = st.loadActive().overrides;
        OfflineTuner.Result r = ParamRegistry.with(active, () -> {
            if (a[1].equals("flywheel")) return OfflineTuner.tuneFlywheel(FlywheelLqrController.Params.fromConfig(), Double.parseDouble(a[2]), 1);
            return OfflineTuner.tuneTurret(TurretStateSpaceController.Params.fromConfig(), 1);
        });
        out.println(OfflineTuner.DISCLAIMER);
        for (String s : r.table) out.println(s);
        Map<String, Double> cand = new TreeMap<>(st.candidateOrActive());
        cand.putAll(r.best);
        List<String> errs = st.saveCandidate(cand, "offline tuner (" + a[1] + ") - SIMULATION result");
        for (String e : errs) out.println("REFUSED: " + e);
        if (errs.isEmpty()) out.println("candidate written (not active): " + r.best + "\nReview: ConfigTool DIR diff");
        return errs.isEmpty() ? 0 : 1;
    }
}
