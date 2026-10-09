package org.firstinspires.ftc.teamcode.calibration;

import org.firstinspires.ftc.teamcode.tuning.ConfigStore;
import org.firstinspires.ftc.teamcode.tuning.Param;
import org.firstinspires.ftc.teamcode.tuning.ParamRegistry;

import java.util.Map;

/**
 * Selection + stepping over a subsystem's tunable keys, editing the session's CANDIDATE only. SAFETY and
 * FIXED_DERIVED parameters are refused here (they are not tunable by hand). Pure logic: the OpModes only map
 * gamepad buttons onto these calls.
 */
public final class ParamEditor {
    private final CalSession session;
    private final String[] keys;
    private int sel;

    public ParamEditor(CalSession session, String... keys) {
        this.session = session;
        this.keys = keys;
        for (String k : keys) {
            Param p = ParamRegistry.get(k);
            if (p == null) throw new IllegalArgumentException("unknown parameter " + k);
            if (!p.manuallyAdjustable()) throw new IllegalArgumentException(k + " is not manually adjustable (" + p.category + ")");
        }
    }

    public String[] keys() { return keys; }
    public int selected() { return sel; }
    public String selectedKey() { return keys[sel]; }
    public void move(int delta) { sel = ((sel + delta) % keys.length + keys.length) % keys.length; }

    /** Adds {@code steps * Param.step} to the selected candidate value (booleans toggle). Returns an error or null. */
    public String adjust(int steps) {
        Param p = ParamRegistry.get(keys[sel]);
        double cur = session.candidateValue(p.key);
        double next;
        if (p.isBool) next = cur != 0 ? 0 : 1;
        else {
            double base = Double.isNaN(cur) ? 0 : cur;
            next = base + steps * p.step;
            next = Math.max(p.min, Math.min(p.max, next));
            next = Math.round(next / (p.step / 1000.0)) * (p.step / 1000.0);   // kill float drift
        }
        return session.setManual(p.key, next);
    }

    public String describe(int i) {
        Param p = ParamRegistry.get(keys[i]);
        double cur = session.candidateValue(p.key), act = session.activeValue(p.key);
        String s = String.format("%s%s = %s %s", i == sel ? "> " : "  ", p.key, p.format(cur), p.unit);
        if (!(Double.isNaN(cur) && Double.isNaN(act)) && cur != act) s += "   (active " + p.format(act) + ")";
        if (p.placeholder && !session.isExplicit(p.key)) s += "  [PLACEHOLDER]";
        return s;
    }
}
