package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/**
 * Class-API base with the builder's defaults. Used where a command needs per-run state (timers, phases);
 * create a NEW instance for each schedule (the factories in {@link RobotCommands} do this).
 */
abstract class BaseCommand implements Command {
    private final Set<Object> requirements;
    private final int priority;
    private final ConflictBehavior conflict;

    BaseCommand(int priority, ConflictBehavior conflict, Object... reqs) {
        Set<Object> s = new HashSet<>();
        Collections.addAll(s, reqs);
        this.requirements = Collections.unmodifiableSet(s);
        this.priority = priority;
        this.conflict = conflict;
    }

    @Override public Set<Object> requirements() { return requirements; }
    @Override public int priority() { return priority; }
    /** END, never SUSPEND: resuming a half-finished feed/spin-up against stale state is unsafe. */
    @Override public InterruptedBehavior interruptedBehavior() { return InterruptedBehavior.END; }
    @Override public BlockedBehavior blockedBehavior() { return BlockedBehavior.CANCEL; }
    @Override public ConflictBehavior conflictBehavior() { return conflict; }
    @Override public void start() {}
    @Override public void execute() {}
    @Override public boolean done() { return false; }
    @Override public void end(EndCondition endCondition) {}

    static double nowMs() { return org.firstinspires.ftc.teamcode.hardware.RobotClock.ms(); }
}
