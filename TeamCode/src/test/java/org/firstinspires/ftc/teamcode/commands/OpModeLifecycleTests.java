package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.info;
import static org.firstinspires.ftc.teamcode.commands.SimSupport.*;

import java.io.File;
import java.nio.file.Files;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * STATIC SOURCE CHECKS of every OpMode's exit path. OpModes cannot be instantiated on the host (they need the FTC
 * runtime), so this verifies the code text, NOT behaviour: an OpMode is not reported as physically tested by this.
 */
public final class OpModeLifecycleTests {
    private OpModeLifecycleTests() {}

    public static void run() {
        sim("opmode exit paths (source scan): every OpMode shuts hardware down explicitly after Scheduler.reset()", s -> {
            File root = new File("TeamCode/src/main/java/org/firstinspires/ftc/teamcode");
            int checked = 0;
            for (File f : StateSafetyTests.listJava(root)) {
                String src = new String(Files.readAllBytes(f.toPath()));
                boolean iterative = src.matches("(?s).*class\\s+\\w+\\s+extends\\s+OpMode\\b.*");
                boolean linear = src.matches("(?s).*class\\s+\\w+\\s+extends\\s+(LinearOpMode|CalibrationOpMode)\\b.*");
                if (!iterative && !linear) continue;
                checked++;
                if (iterative) {
                    Matcher m = Pattern.compile("public void stop\\(\\)\\s*\\{").matcher(src);
                    check(m.find(), f.getName() + ": iterative OpMode must override stop()");
                    String body = braceBody(src, m.end());
                    int reset = body.indexOf("Scheduler.reset()"), shut = body.indexOf("safeShutdown(");
                    check(reset >= 0 && shut > reset, f.getName() + ": stop() must call Scheduler.reset() THEN robot.safeShutdown(...)");
                } else if (!f.getName().equals("CalibrationOpMode.java")) {
                    check(src.contains("extends CalibrationOpMode"), f.getName() + ": calibration OpModes extend the shared base that guarantees cleanup");
                } else {
                    Matcher m = Pattern.compile("finally\\s*\\{").matcher(src);
                    boolean ok = false;
                    while (m.find()) if (braceBody(src, m.end()).contains("safeShutdown(")) ok = true;
                    check(ok, f.getName() + ": base runOpMode must call safeShutdown in a finally block");
                }
            }
            info("scanned " + checked + " OpMode source files");
            check(checked >= 8, "found the teleop, the example auto and the calibration OpModes: " + checked);
        });
    }

    /** Text between an opening brace (already consumed at {@code from}) and its matching close. */
    static String braceBody(String src, int from) {
        int depth = 1, i = from;
        while (i < src.length() && depth > 0) { char ch = src.charAt(i++); if (ch == '{') depth++; else if (ch == '}') depth--; }
        return src.substring(from, i);
    }
}
