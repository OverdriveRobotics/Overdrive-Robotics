package org.firstinspires.ftc.teamcode.tuning;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * Production-side hook: applies the accepted ACTIVE configuration to the live parameters at OpMode init.
 * A missing file means "use compiled defaults". An invalid file is rejected as a whole (defaults stay) and the
 * reasons are returned for telemetry - a half-applied configuration is never used.
 */
public final class Tuning {
    private Tuning() {}

    public static volatile int activeRevision = 0;
    public static volatile String activeHash = "";
    public static volatile List<String> loadProblems = new ArrayList<>();

    /** Directory on the robot controller (AppUtil.FIRST_FOLDER/overdrive). */
    public static File robotDir(File firstFolder) { return new File(firstFolder, "overdrive"); }

    public static List<String> applyActive(File dir) {
        List<String> problems = new ArrayList<>();
        org.firstinspires.ftc.teamcode.hardware.Storage.FieldPoint keepA = org.firstinspires.ftc.teamcode.hardware.Storage.targetA,
                keepB = org.firstinspires.ftc.teamcode.hardware.Storage.targetB;
        try {
            ParamRegistry.resetToDefaults();
            ConfigStore.Config c = new ConfigStore(dir).loadActive();
            problems.addAll(ConfigValidator.validate(c.overrides));
            if (problems.isEmpty()) {
                ParamRegistry.apply(c.overrides);
                // Target points set by an OpMode survive unless the accepted file defines them.
                if (!c.overrides.containsKey("target.A_X") && !c.overrides.containsKey("target.A_Y")) org.firstinspires.ftc.teamcode.hardware.Storage.targetA = keepA;
                if (!c.overrides.containsKey("target.B_X") && !c.overrides.containsKey("target.B_Y")) org.firstinspires.ftc.teamcode.hardware.Storage.targetB = keepB;
                activeRevision = c.revision;
                activeHash = c.hash();
            }
        } catch (Exception e) {
            problems.add("could not load active configuration: " + e.getMessage());
        }
        if (!problems.isEmpty()) {
            ParamRegistry.resetToDefaults();
            org.firstinspires.ftc.teamcode.hardware.Storage.targetA = keepA;
            org.firstinspires.ftc.teamcode.hardware.Storage.targetB = keepB;
            activeRevision = 0;
            activeHash = ConfigStore.hash(ConfigStore.effective(new java.util.TreeMap<String, Double>()));
        }
        loadProblems = problems;
        return problems;
    }
}
