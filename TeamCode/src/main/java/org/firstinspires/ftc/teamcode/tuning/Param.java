package org.firstinspires.ftc.teamcode.tuning;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

/**
 * One tunable value, bound to the production static field (or accessor) that the controllers actually read, so
 * calibration, tests and the robot all operate on the same definitions. Values are exchanged as doubles
 * (booleans 0/1, ints rounded).
 */
public final class Param {
    public enum Category {
        /** Established geometry / verified specs. Never adjusted by tools. */
        FIXED_DERIVED,
        /** Encoder offsets, conversion factors, measured mechanism properties. Set from measurements. */
        HARDWARE_CAL,
        /** Gains, weights, feedforward. */
        CONTROLLER,
        /** Tolerances, timings, windows, timeouts. */
        BEHAVIOR,
        /** Output caps, sanity bounds. Not adjustable by the interactive tools; file edits may only TIGHTEN them. */
        SAFETY,
        /** Master enables guarding unvalidated hardware paths; validator demands the dependent values be explicit. */
        INTERLOCK
    }

    public interface Accessor { double get(); void set(double v); }

    public final String key, unit, description;
    public final Category category;
    public final double min, max, step;
    /** True while the compiled default is a placeholder, not hardware data. */
    public final boolean placeholder;
    public final boolean isInt, isBool;
    /** SAFETY only: LOWER = file may only reduce vs the compiled default, HIGHER = only increase, NONE = measured-only. */
    public enum Tighten { NONE, LOWER, HIGHER }
    public final Tighten tighten;
    private final Accessor accessor;

    Param(String key, Accessor accessor, boolean isInt, boolean isBool, Category cat, double min, double max,
          double step, String unit, String description, boolean placeholder, Tighten tighten) {
        this.key = key; this.accessor = accessor; this.isInt = isInt; this.isBool = isBool; this.category = cat;
        this.min = min; this.max = max; this.step = step; this.unit = unit; this.description = description;
        this.placeholder = placeholder; this.tighten = tighten;
    }

    public double get() { return accessor.get(); }

    /** Sets the live value after range/type validation. Throws IllegalArgumentException when invalid. */
    public void set(double v) {
        String err = check(v);
        if (err != null) throw new IllegalArgumentException(key + ": " + err);
        accessor.set(isBool ? (v != 0 ? 1 : 0) : isInt ? Math.rint(v) : v);
    }

    /** Unchecked write used to restore previously-live values (including NaN defaults). */
    void rawSet(double v) { accessor.set(v); }

    /** Null if v is acceptable for this parameter, otherwise the reason. */
    public String check(double v) {
        if (Double.isNaN(v)) return "NaN";
        if (isBool && v != 0 && v != 1) return "must be 0 or 1";
        if (isInt && v != Math.rint(v)) return "must be an integer";
        if (v < min || v > max) return "out of range [" + min + ", " + max + "]: " + v;
        return null;
    }

    /** Whether the interactive tools may change this value by hand. */
    public boolean manuallyAdjustable() { return category != Category.FIXED_DERIVED && category != Category.SAFETY; }

    public String format(double v) {
        if (isBool) return v != 0 ? "true" : "false";
        if (isInt) return Long.toString(Math.round(v));
        return Double.toString(v);
    }

    /** Static-field accessor (field must be public static and non-final). */
    static Accessor field(Class<?> c, String name) {
        try {
            final Field f = c.getField(name);
            if (!Modifier.isStatic(f.getModifiers()) || Modifier.isFinal(f.getModifiers()))
                throw new IllegalStateException(c.getSimpleName() + "." + name + " must be public static non-final");
            final Class<?> t = f.getType();
            return new Accessor() {
                @Override public double get() {
                    try {
                        return t == boolean.class ? (f.getBoolean(null) ? 1 : 0) : t == int.class ? f.getInt(null) : f.getDouble(null);
                    } catch (IllegalAccessException e) { throw new IllegalStateException(e); }
                }
                @Override public void set(double v) {
                    try {
                        if (t == boolean.class) f.setBoolean(null, v != 0);
                        else if (t == int.class) f.setInt(null, (int) Math.rint(v));
                        else f.setDouble(null, v);
                    } catch (IllegalAccessException e) { throw new IllegalStateException(e); }
                }
            };
        } catch (NoSuchFieldException e) {
            throw new IllegalStateException("no such config field " + c.getSimpleName() + "." + name);
        }
    }

    static boolean fieldIsInt(Class<?> c, String name) { try { return c.getField(name).getType() == int.class; } catch (Exception e) { return false; } }
    static boolean fieldIsBool(Class<?> c, String name) { try { return c.getField(name).getType() == boolean.class; } catch (Exception e) { return false; } }
}
