package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

/** Tiny assertion/runner helper. JUnit is not in the offline Gradle cache, so tests run via scripts/run_tests.sh. */
public final class T {
    private T() {}

    public interface Body { void run() throws Exception; }

    public static int passed, failed;
    public static final List<String> failures = new ArrayList<>();
    private static String current = "";

    public static void test(String name, Body b) {
        current = name;
        try {
            b.run();
            passed++;
            System.out.println("PASS  " + name);
        } catch (Throwable t) {
            failed++;
            failures.add(name + ": " + t);
            System.out.println("FAIL  " + name + "  -> " + t);
            if (!(t instanceof AssertionError)) t.printStackTrace(System.out);
        }
    }

    public static void check(boolean cond, String msg) { if (!cond) throw new AssertionError(msg); }
    public static void near(double expected, double actual, double tol, String msg) {
        if (!(Math.abs(expected - actual) <= tol)) throw new AssertionError(msg + ": expected " + expected + " got " + actual);
    }
    public static void info(String s) { System.out.println("      [" + current + "] " + s); }
}
