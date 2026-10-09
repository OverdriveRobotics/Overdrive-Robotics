package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.calibration.CalContext;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

/** CalContext backed by the simulator and a scripted "operator". Everything it produces is labelled SIMULATION. */
public class SimCalContext implements CalContext {
    /** Scripted operator. */
    public interface Operator {
        boolean confirm(String prompt, SimRobot s);
        double value(String prompt, double initial, SimRobot s);
    }

    public final SimRobot sim;
    public Operator operator = new Operator() {
        @Override public boolean confirm(String prompt, SimRobot s) { return true; }
        @Override public double value(String prompt, double initial, SimRobot s) { return initial; }
    };
    /** Called every loop before the step (e.g. to move the robot by "hand"). */
    public Consumer<SimRobot> onLoop;
    public double abortAtSec = Double.POSITIVE_INFINITY;
    public final List<String> prompts = new ArrayList<>();
    public final List<String> statuses = new ArrayList<>();
    public boolean echo;
    private boolean aborted;

    public SimCalContext(SimRobot sim) { this.sim = sim; }

    @Override public RobotHardware robot() { return sim.robot; }
    @Override public String mode() { return "SIMULATION"; }
    @Override public void tick() {
        if (onLoop != null) onLoop.accept(sim);
        sim.step(10);
    }
    @Override public double timeSec() { return sim.nanos / 1e9; }
    @Override public boolean abortRequested() { return aborted || timeSec() >= abortAtSec; }
    public void abort() { aborted = true; }
    @Override public boolean confirm(String prompt) {
        prompts.add(prompt);
        if (echo) System.out.println("      ? " + prompt);
        return operator.confirm(prompt, sim);
    }
    @Override public double promptValue(String prompt, double initial, double step, double min, double max) {
        prompts.add(prompt);
        double v = operator.value(prompt, initial, sim);
        return Math.max(min, Math.min(max, v));
    }
    @Override public void status(String line) { statuses.add(line); if (echo) System.out.println("      > " + line); }
}
