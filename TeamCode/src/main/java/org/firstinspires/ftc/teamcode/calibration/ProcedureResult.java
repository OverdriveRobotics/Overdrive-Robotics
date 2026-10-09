package org.firstinspires.ftc.teamcode.calibration;

import org.firstinspires.ftc.teamcode.tuning.RunLog;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/** What a procedure produced: log, metrics, proposed parameter values (NOT applied) and notes. */
public final class ProcedureResult {
    public RunLog log;
    public final Map<String, Double> metrics = new TreeMap<>();
    /** Proposed values; the operator decides whether they enter the candidate. */
    public final Map<String, Double> suggestions = new LinkedHashMap<>();
    public final List<String> notes = new ArrayList<>();
    public boolean aborted;
    public boolean refused;   // preflight failed: nothing was run

    public static ProcedureResult refused(List<String> reasons) {
        ProcedureResult r = new ProcedureResult();
        r.refused = true;
        r.notes.addAll(reasons);
        return r;
    }
}
