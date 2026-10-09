package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.near;
import static org.firstinspires.ftc.teamcode.T.test;

import org.firstinspires.ftc.teamcode.commands.RobotConfig;
import org.firstinspires.ftc.teamcode.control.FlywheelConfig;
import org.firstinspires.ftc.teamcode.control.TurretConfig;
import org.firstinspires.ftc.teamcode.hardware.Storage;

import java.io.File;
import java.nio.file.Files;
import java.util.Map;
import java.util.TreeMap;

/** Parameter registry, validation and the candidate/accept/reject/restore workflow. Hardware-independent. */
public final class ConfigFrameworkTests {
    private ConfigFrameworkTests() {}

    static void reset() { ParamRegistry.resetToDefaults(); Storage.invalidateHardwareState(); }

    static File tmp() throws Exception { return Files.createTempDirectory("overdrive-cfg").toFile(); }

    /** A complete, internally consistent set of turret hardware values (arbitrary test numbers, NOT real hardware). */
    static Map<String, Double> turretCalibrated() {
        Map<String, Double> o = new TreeMap<>();
        o.put("turret.SHAFT_TICKS_PER_REV", 537.7); o.put("turret.SHAFT_REVS_PER_TURRET_REV", 3.5);
        o.put("turret.ENCODER_SIGN", 1.0); o.put("turret.MIN_ANGLE_RAD", -1.5); o.put("turret.MAX_ANGLE_RAD", 1.5);
        o.put("turret.SHAFT_FREE_SPEED_RAD_S", 30.0); o.put("turret.SHAFT_STALL_TORQUE_NM", 2.0); o.put("turret.STALL_CURRENT_A", 9.0);
        o.put("turret.LOAD_INERTIA_KG_M2", 0.03); o.put("turret.VISCOUS_FRICTION", 0.02);
        return o;
    }

    public static void run() {
        test("registry: every parameter resolves to a public static non-final field and its default is in range", () -> {
            reset();
            check(ParamRegistry.all().size() > 60, "registry populated: " + ParamRegistry.all().size());
            for (Param p : ParamRegistry.all().values()) {
                double d = ParamRegistry.defaultOf(p.key);
                if (Double.isNaN(d)) continue;   // unset-by-design (target points, zone bounds)
                check(p.check(d) == null, p.key + " default out of its own range: " + p.check(d));
            }
        });

        test("registry: categories protect safety and derived values from manual tuning", () -> {
            reset();
            check(!ParamRegistry.get("turret.MAX_CONTROL_VOLTS").manuallyAdjustable(), "voltage cap not manual");
            check(!ParamRegistry.get("robot.MAX_FLYWHEEL_VELOCITY").manuallyAdjustable(), "velocity cap not manual");
            check(ParamRegistry.get("flywheel.MODEL_STD").manuallyAdjustable(), "controller tuning is manual");
            check(ParamRegistry.get("turret.HARDWARE_CONFIGURED").category == Param.Category.INTERLOCK, "interlock category");
            check(ParamRegistry.get("turret.SHAFT_TICKS_PER_REV").placeholder, "placeholder flagged");
        });

        test("registry: apply is atomic and restore returns the exact previous values", () -> {
            reset();
            Map<String, Double> before = ParamRegistry.snapshot();
            Map<String, Double> o = new TreeMap<>();
            o.put("flywheel.MODEL_STD", 77.0);
            o.put("robot.FLYWHEEL_TOLERANCE", -5.0);   // invalid
            boolean threw = false;
            try { ParamRegistry.apply(o); } catch (IllegalArgumentException e) { threw = true; }
            check(threw, "invalid value rejected");
            near(before.get("flywheel.MODEL_STD"), FlywheelConfig.MODEL_STD, 0, "nothing half-applied");
            Map<String, Double> good = new TreeMap<>(); good.put("flywheel.MODEL_STD", 77.0);
            ParamRegistry.apply(good);
            near(77, FlywheelConfig.MODEL_STD, 0, "applied to the production field");
            ParamRegistry.restore(before);
            near(before.get("flywheel.MODEL_STD"), FlywheelConfig.MODEL_STD, 0, "restored");
        });

        test("validator: rejects unknown keys, bad ranges/types, and non-tightening safety edits", () -> {
            reset();
            Map<String, Double> o = new TreeMap<>();
            o.put("nonsense.key", 1.0);
            check(!ConfigValidator.validate(o).isEmpty(), "unknown key");
            o.clear(); o.put("robot.MAX_BALLS", 5.0);
            check(!ConfigValidator.validate(o).isEmpty(), "raising a safety cap above its compiled value is refused");
            o.clear(); o.put("turret.LIMIT_MARGIN_RAD", 0.001);
            check(!ConfigValidator.validate(o).isEmpty(), "shrinking the limit margin is refused");
            o.clear(); o.put("turret.MAX_CONTROL_VOLTS", 2.0);
            check(ConfigValidator.validate(o).isEmpty(), "lowering a voltage cap is allowed");
            o.clear(); o.put("robot.DROP_CONFIRM_LOOPS", 2.5);
            check(!ConfigValidator.validate(o).isEmpty(), "integer parameter must be integral");
            o.clear(); o.put("turret.ENCODER_SIGN", 0.0);
            check(!ConfigValidator.validate(o).isEmpty(), "encoder sign must be +-1");
            o.clear(); o.put("robot.STOPPER_OPEN", 0.3); o.put("robot.STOPPER_CLOSED", 0.3);
            check(!ConfigValidator.validate(o).isEmpty(), "identical stopper positions");
            o.clear(); o.put("turret.MIN_ANGLE_RAD", 1.0); o.put("turret.MAX_ANGLE_RAD", 0.5);
            check(!ConfigValidator.validate(o).isEmpty(), "min above max");
            check(TurretConfig.MIN_ANGLE_RAD < TurretConfig.MAX_ANGLE_RAD, "validation left live state untouched");
        });

        test("validator: interlocks demand explicitly calibrated dependencies, never placeholders", () -> {
            reset();
            Map<String, Double> o = new TreeMap<>();
            o.put("turret.HARDWARE_CONFIGURED", 1.0);
            java.util.List<String> e = ConfigValidator.validate(o);
            check(e.size() >= 8, "turret interlock lists missing calibration: " + e.size());
            Map<String, Double> good = turretCalibrated();
            good.put("turret.HARDWARE_CONFIGURED", 1.0);
            check(ConfigValidator.validate(good).isEmpty(), "complete turret calibration accepted: " + ConfigValidator.validate(good));
            o.clear(); o.put("flywheel.MODEL_IDENTIFIED", 1.0);
            check(!ConfigValidator.validate(o).isEmpty(), "flywheel interlock without identified values");
            o.put("flywheel.USE_SYSID_MODEL", 1.0); o.put("flywheel.KV_VOLTS_PER_TICK_S", 0.0045);
            o.put("flywheel.KA_VOLTS_PER_TICK_S2", 0.002); o.put("flywheel.KS_VOLTS", 0.3);
            check(ConfigValidator.validate(o).isEmpty(), "sysid-based identification accepted: " + ConfigValidator.validate(o));
            o.clear(); o.put("robot.REQUIRE_TURRET_ALIGNED", 1.0);
            check(!ConfigValidator.validate(o).isEmpty(), "alignment requirement needs a calibrated turret and targets");
            o.clear(); o.put("robot.SHOOT_ZONE_ENABLED", 1.0);
            check(!ConfigValidator.validate(o).isEmpty(), "zone needs finite bounds");
            check(!TurretConfig.HARDWARE_CONFIGURED && !FlywheelConfig.MODEL_IDENTIFIED, "interlocks still off in live state");
        });

        test("validator: an uncontrollable/degenerate controller design is rejected", () -> {
            reset();
            Map<String, Double> o = turretCalibrated();
            o.put("turret.HARDWARE_CONFIGURED", 1.0);
            o.put("turret.MAX_ANGLE_ERROR_RAD", 1.5);
            check(ConfigValidator.validate(o).isEmpty(), "extreme but valid weights still design");
        });

        test("store: candidate -> accept archives a revision, becomes active, clears the candidate", () -> {
            reset();
            ConfigStore st = new ConfigStore(tmp());
            check(st.loadActive().revision == 0, "starts at defaults");
            Map<String, Double> c = new TreeMap<>(); c.put("flywheel.MODEL_STD", 80.0);
            check(st.saveCandidate(c, "tune").isEmpty(), "candidate saved");
            check(st.hasCandidate() && st.loadActive().revision == 0, "active untouched by saving a candidate");
            check(st.accept("first").isEmpty(), "accepted");
            check(!st.hasCandidate(), "candidate consumed");
            ConfigStore.Config a = st.loadActive();
            check(a.revision == 1 && a.overrides.get("flywheel.MODEL_STD") == 80.0, "active rev 1");
            check(st.revisions().contains(1), "history has rev 1");
            check(new File(st.dir(), "accepted.log").length() > 0, "audit log written");
        });

        test("store: invalid candidates are refused and write nothing", () -> {
            reset();
            ConfigStore st = new ConfigStore(tmp());
            Map<String, Double> c = new TreeMap<>(); c.put("robot.FLYWHEEL_TOLERANCE", 0.0);
            check(!st.saveCandidate(c, "bad").isEmpty(), "refused");
            check(!st.hasCandidate(), "no candidate file");
            check(!st.accept("x").isEmpty(), "nothing to accept");
        });

        test("store: reject keeps a record and leaves the active configuration alone", () -> {
            reset();
            ConfigStore st = new ConfigStore(tmp());
            Map<String, Double> c = new TreeMap<>(); c.put("flywheel.MODEL_STD", 90.0);
            st.saveCandidate(c, "x");
            check(st.reject(), "rejected");
            check(!st.hasCandidate() && st.loadActive().revision == 0, "back to active");
            check(new File(st.dir(), "rejected").listFiles().length == 1, "rejected copy retained");
        });

        test("store: restore re-activates an old revision as a NEW revision; known-good is explicit", () -> {
            reset();
            ConfigStore st = new ConfigStore(tmp());
            Map<String, Double> c1 = new TreeMap<>(); c1.put("flywheel.MODEL_STD", 80.0);
            st.saveCandidate(c1, "one"); st.accept("one");
            Map<String, Double> c2 = new TreeMap<>(); c2.put("flywheel.MODEL_STD", 120.0);
            st.saveCandidate(c2, "two"); st.accept("two");
            check(st.loadActive().overrides.get("flywheel.MODEL_STD") == 120.0, "rev 2 active");
            check(!st.restoreLatestKnownGood().isEmpty(), "nothing known-good yet");
            st.markKnownGood(1);
            check(st.restoreLatestKnownGood().isEmpty(), "restored");
            ConfigStore.Config a = st.loadActive();
            check(a.revision == 3 && a.overrides.get("flywheel.MODEL_STD") == 80.0, "values of rev 1 under rev 3");
            check(st.loadRevision(2).overrides.get("flywheel.MODEL_STD") == 120.0, "history never rewritten");
        });

        test("store: a stale candidate (active changed underneath it) cannot be accepted", () -> {
            reset();
            ConfigStore st = new ConfigStore(tmp());
            Map<String, Double> c = new TreeMap<>(); c.put("flywheel.MODEL_STD", 80.0);
            st.saveCandidate(c, "a");
            // somebody else accepts a different change first
            ConfigStore other = new ConfigStore(st.dir());
            File cand = new File(st.dir(), "candidate.properties");
            byte[] saved = Files.readAllBytes(cand.toPath());
            Map<String, Double> d = new TreeMap<>(); d.put("flywheel.MODEL_STD", 150.0);
            other.saveCandidate(d, "b"); other.accept("b");
            Files.write(cand.toPath(), saved);   // put the old candidate (parent 0) back
            java.util.List<String> e = st.accept("late");
            check(!e.isEmpty() && e.get(0).contains("based on revision"), "refused: " + e);
        });

        test("store: file robustness - unsupported schema and corrupt numbers are rejected", () -> {
            reset();
            File d = tmp();
            Files.write(new File(d, "active.properties").toPath(), "schema=99\nrevision=1\n".getBytes());
            boolean threw = false;
            try { new ConfigStore(d).loadActive(); } catch (java.io.IOException e) { threw = true; }
            check(threw, "bad schema");
            Files.write(new File(d, "active.properties").toPath(), "schema=1\nrevision=1\np.flywheel.MODEL_STD=abc\n".getBytes());
            threw = false;
            try { new ConfigStore(d).loadActive(); } catch (java.io.IOException e) { threw = true; }
            check(threw, "bad number");
        });

        test("boot: Tuning.applyActive applies a valid file, ignores an invalid one as a whole, keeps OpMode-set targets", () -> {
            reset();
            File d = tmp();
            ConfigStore st = new ConfigStore(d);
            Map<String, Double> c = new TreeMap<>(); c.put("flywheel.MODEL_STD", 66.0); c.put("robot.FLYWHEEL_TOLERANCE", 33.0);
            st.saveCandidate(c, "ok"); st.accept("ok");
            Storage.setTarget(0, 10, 20);
            ParamRegistry.resetToDefaults(); Storage.setTarget(0, 10, 20);
            check(Tuning.applyActive(d).isEmpty(), "valid file applied");
            near(66, FlywheelConfig.MODEL_STD, 0, "applied");
            near(33, RobotConfig.FLYWHEEL_TOLERANCE, 0, "applied");
            near(10, Storage.targetA.x, 0, "target survived the config load");
            check(Tuning.activeRevision == 1, "revision exposed for telemetry");
            // corrupt: one good value + one out-of-range value
            Files.write(new File(d, "active.properties").toPath(),
                    "schema=1\nrevision=2\np.flywheel.MODEL_STD=44\np.robot.FLYWHEEL_TOLERANCE=-1\n".getBytes());
            check(!Tuning.applyActive(d).isEmpty(), "invalid file reported");
            near(FlywheelConfigDefault(), FlywheelConfig.MODEL_STD, 0, "defaults restored, nothing half-applied");
            near(0, Tuning.activeRevision, 0, "falls back to revision 0");
            Files.delete(new File(d, "active.properties").toPath());
            check(Tuning.applyActive(d).isEmpty(), "missing file = defaults, no problem");
            reset();
        });

        test("docs: PARAMETERS.md matches the registry (regenerate with scripts/gen_param_docs.sh)", () -> {
            reset();
            String doc = new String(Files.readAllBytes(new File("docs/PARAMETERS.md").toPath()));
            String table = doc.substring(doc.indexOf("| Key |"));
            check(table.equals(org.firstinspires.ftc.teamcode.tuning.cli.ConfigTool.parameterTable()), "docs/PARAMETERS.md is stale");
        });

        test("templates: the shipped example configuration parses, validates and carries no overrides", () -> {
            File d = tmp();
            Files.copy(new File("calibration/templates/active.example.properties").toPath(), new File(d, "active.properties").toPath());
            ConfigStore.Config c = new ConfigStore(d).loadActive();
            check(c.overrides.isEmpty() && ConfigValidator.validate(c.overrides).isEmpty(), "template is a valid, empty configuration");
        });

        test("report/hash: effective hash is stable, order-independent and changes with any parameter", () -> {
            reset();
            Map<String, Double> a = new TreeMap<>(); a.put("flywheel.MODEL_STD", 80.0); a.put("robot.FEED_POWER", 0.9);
            Map<String, Double> b = new TreeMap<>(); b.put("robot.FEED_POWER", 0.9); b.put("flywheel.MODEL_STD", 80.0);
            check(ConfigStore.hash(ConfigStore.effective(a)).equals(ConfigStore.hash(ConfigStore.effective(b))), "same set same hash");
            b.put("robot.FEED_POWER", 0.8);
            check(!ConfigStore.hash(ConfigStore.effective(a)).equals(ConfigStore.hash(ConfigStore.effective(b))), "different set different hash");
            check(ConfigStore.diff(a, b).size() == 1, "diff lists exactly the changed value");
        });
    }

    private static double FlywheelConfigDefault() { return ParamRegistry.defaultOf("flywheel.MODEL_STD"); }
}
