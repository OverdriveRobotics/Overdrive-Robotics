package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.T.check;
import static org.firstinspires.ftc.teamcode.T.test;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;

import java.util.ArrayList;
import java.util.List;

/** Ivy scheduler semantics the robot code relies on, verified against the installed library (1.1.1). Hardware-free. */
public final class IvyTests {
    private IvyTests() {}

    private static final class Probe {
        final List<String> log;
        final String name;
        boolean finish;
        int executes;
        Probe(String name, List<String> log) { this.name = name; this.log = log; }
        Command cmd(Object res, int prio, ConflictBehavior cb) {
            return Command.build().requiring(res).setPriority(prio).setConflictBehavior(cb)
                    .setStart(() -> log.add(name + ":start"))
                    .setExecute(() -> executes++)
                    .setDone(() -> finish)
                    .setEnd(e -> log.add(name + ":end:" + e));
        }
    }

    private static void spin(int n) { for (int i = 0; i < n; i++) Scheduler.execute(); }

    private static void fresh(Runnable r) { Scheduler.reset(); r.run(); Scheduler.reset(); }

    public static void run() {
        test("ivy: sequential runs children in order, one after another", () -> fresh(() -> {
            List<String> log = new ArrayList<>();
            Command seq = sequential(instant(() -> log.add("a")), instant(() -> log.add("b")), instant(() -> log.add("c")));
            Scheduler.schedule(seq);
            spin(6);
            check(log.equals(java.util.Arrays.asList("a", "b", "c")), "order " + log);
            check(!Scheduler.isScheduled(seq), "group finished");
        }));

        test("ivy: parallel runs children in the same loop and ends when ALL end", () -> fresh(() -> {
            List<String> log = new ArrayList<>();
            Probe a = new Probe("a", log), b = new Probe("b", log);
            Command p = parallel(a.cmd(new Object(), 0, ConflictBehavior.OVERRIDE), b.cmd(new Object(), 0, ConflictBehavior.OVERRIDE));
            Scheduler.schedule(p);
            spin(3);
            check(a.executes >= 2 && b.executes >= 2, "both executing concurrently");
            a.finish = true; spin(3);
            check(Scheduler.isScheduled(p), "still running while b runs");
            b.finish = true; spin(3);
            check(!Scheduler.isScheduled(p), "ended when both ended");
        }));

        test("ivy: race ends with the first and interrupts the rest; deadline ends with its deadline command", () -> fresh(() -> {
            List<String> log = new ArrayList<>();
            Probe slow = new Probe("slow", log), fast = new Probe("fast", log);
            Command r = race(slow.cmd(new Object(), 0, ConflictBehavior.OVERRIDE), fast.cmd(new Object(), 0, ConflictBehavior.OVERRIDE));
            Scheduler.schedule(r);
            spin(2);
            fast.finish = true; spin(3);
            check(!Scheduler.isScheduled(r), "race over");
            check(log.contains("slow:end:INTERRUPTED") && log.contains("fast:end:NATURALLY"), "loser interrupted, winner natural: " + log);
            log.clear();
            Probe dl = new Probe("dl", log), bg = new Probe("bg", log);
            Command d = deadline(dl.cmd(new Object(), 0, ConflictBehavior.OVERRIDE), bg.cmd(new Object(), 0, ConflictBehavior.OVERRIDE));
            Scheduler.schedule(d);
            spin(2);
            dl.finish = true; spin(3);
            check(!Scheduler.isScheduled(d) && log.contains("bg:end:INTERRUPTED"), "background cancelled when the deadline ended: " + log);
        }));

        test("ivy: equal priority + OVERRIDE interrupts the running command (its end() runs); CANCEL refuses the newcomer", () -> fresh(() -> {
            List<String> log = new ArrayList<>();
            Object res = new Object();
            Probe a = new Probe("a", log), b = new Probe("b", log), c = new Probe("c", log);
            Command ca = a.cmd(res, 0, ConflictBehavior.OVERRIDE), cb = b.cmd(res, 0, ConflictBehavior.OVERRIDE);
            Scheduler.schedule(ca); spin(1);
            Scheduler.schedule(cb); spin(1);
            check(log.contains("a:end:INTERRUPTED") && Scheduler.isScheduled(cb) && !Scheduler.isScheduled(ca), "override: " + log);
            Command cc = c.cmd(res, 0, ConflictBehavior.CANCEL);
            Scheduler.schedule(cc); spin(2);
            check(!Scheduler.isScheduled(cc) && !log.contains("c:start") && Scheduler.isScheduled(cb), "cancel: newcomer never started");
        }));

        test("ivy: ConflictBehavior.QUEUE waits for the resource and then starts", () -> fresh(() -> {
            List<String> log = new ArrayList<>();
            Object res = new Object();
            Probe a = new Probe("a", log), b = new Probe("b", log);
            Command ca = a.cmd(res, 0, ConflictBehavior.OVERRIDE), cb = b.cmd(res, 0, ConflictBehavior.QUEUE);
            Scheduler.schedule(ca); spin(1);
            Scheduler.schedule(cb); spin(2);
            check(!log.contains("b:start") && Scheduler.isScheduled(cb), "queued, not started: " + log);
            a.finish = true; spin(4);
            check(log.contains("b:start"), "started after the resource freed: " + log);
        }));

        test("ivy: priorities - higher interrupts lower; lower is blocked (CANCEL) or queued (QUEUE)", () -> fresh(() -> {
            List<String> log = new ArrayList<>();
            Object res = new Object();
            Probe low = new Probe("low", log), high = new Probe("high", log), low2 = new Probe("low2", log), low3 = new Probe("low3", log);
            Command cl = low.cmd(res, 0, ConflictBehavior.OVERRIDE);
            Command ch = high.cmd(res, 5, ConflictBehavior.CANCEL);
            Scheduler.schedule(cl); spin(1);
            Scheduler.schedule(ch); spin(1);
            check(log.contains("low:end:INTERRUPTED") && Scheduler.isScheduled(ch), "high took over");
            Command blocked = Command.build().requiring(res).setPriority(0).setBlockedBehavior(BlockedBehavior.CANCEL).setStart(() -> log.add("blocked:start"));
            Scheduler.schedule(blocked); spin(2);
            check(!log.contains("blocked:start") && !Scheduler.isScheduled(blocked), "lower priority blocked and cancelled");
            Command queued = Command.build().requiring(res).setPriority(0).setBlockedBehavior(BlockedBehavior.QUEUE).setStart(() -> log.add("queued:start")).setDone(() -> true);
            Scheduler.schedule(queued); spin(2);
            check(!log.contains("queued:start"), "queued while blocked");
            high.finish = true; spin(4);
            check(log.contains("queued:start"), "ran after the blocker finished: " + log);
        }));

        test("ivy: InterruptedBehavior.SUSPEND pauses the victim (end(SUSPENDED)) and resumes it later; END never resumes", () -> fresh(() -> {
            List<String> log = new ArrayList<>();
            Object res = new Object();
            Probe v = new Probe("victim", log), hi = new Probe("hi", log);
            Command victim = v.cmd(res, 0, ConflictBehavior.OVERRIDE);
            Command suspendable = Command.build().requiring(res).setPriority(0).setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                    .setStart(() -> log.add("sus:start")).setExecute(() -> log.add("sus:exec")).setEnd(e -> log.add("sus:end:" + e)).setDone(() -> false);
            Scheduler.schedule(suspendable); spin(1);
            Command high = hi.cmd(res, 3, ConflictBehavior.CANCEL);
            Scheduler.schedule(high); spin(1);
            check(log.contains("sus:end:SUSPENDED"), "suspended: " + log);
            hi.finish = true; spin(4);
            check(Scheduler.isScheduled(suspendable), "resumed after the interrupter finished");
            // BaseCommand (robot commands) uses END: never resumed
            Command endBehaviour = Command.build().requiring(res).setPriority(0).setEnd(e -> log.add("end-behaviour:" + e)).setDone(() -> false);
            Scheduler.reset();
            Scheduler.schedule(endBehaviour); spin(1);
            Probe hi2 = new Probe("hi2", log);
            Scheduler.schedule(hi2.cmd(res, 3, ConflictBehavior.CANCEL)); spin(1);
            hi2.finish = true; spin(4);
            check(!Scheduler.isScheduled(endBehaviour) && log.contains("end-behaviour:INTERRUPTED"), "END: terminated, not resumed");
        }));

        test("ivy: groups own the union of their children's requirements and conflict as a unit", () -> fresh(() -> {
            List<String> log = new ArrayList<>();
            Object r1 = new Object(), r2 = new Object();
            Probe a = new Probe("a", log), b = new Probe("b", log), c = new Probe("c", log);
            Command group = parallel(a.cmd(r1, 0, ConflictBehavior.OVERRIDE), b.cmd(r2, 0, ConflictBehavior.OVERRIDE));
            check(group.requirements().contains(r1) && group.requirements().contains(r2), "union of requirements");
            Scheduler.schedule(group); spin(2);
            Scheduler.schedule(c.cmd(r2, 0, ConflictBehavior.OVERRIDE)); spin(2);
            check(!Scheduler.isScheduled(group) && log.contains("a:end:INTERRUPTED") && log.contains("b:end:INTERRUPTED"), "interrupting one resource ends the whole group: " + log);
        }));

        test("ivy: cancelling a group runs end() on every running child", () -> fresh(() -> {
            List<String> log = new ArrayList<>();
            Probe a = new Probe("a", log), b = new Probe("b", log);
            Command seq = sequential(a.cmd(new Object(), 0, ConflictBehavior.OVERRIDE), b.cmd(new Object(), 0, ConflictBehavior.OVERRIDE));
            Scheduler.schedule(seq); spin(2);
            Scheduler.cancel(seq);
            check(log.contains("a:end:INTERRUPTED") && !log.contains("b:start"), "only the running child ended: " + log);
            check(!Scheduler.isScheduled(seq), "gone");
        }));

        test("ivy: nested scheduling - a command may schedule another from start() and from execute()", () -> fresh(() -> {
            List<String> log = new ArrayList<>();
            Command inner1 = instant(() -> log.add("inner1")), inner2 = instant(() -> log.add("inner2"));
            Command outer = Command.build().setStart(() -> Scheduler.schedule(inner1)).setExecute(() -> { if (!log.contains("inner2")) Scheduler.schedule(inner2); })
                    .setDone(() -> log.contains("inner1") && log.contains("inner2"));
            Scheduler.schedule(outer); spin(6);
            check(log.contains("inner1") && log.contains("inner2") && !Scheduler.isScheduled(outer), "both ran: " + log);
        }));

        test("ivy: timeouts via race(waitMs) and until(); waitUntil gates a sequence", () -> fresh(() -> {
            List<String> log = new ArrayList<>();
            Probe never = new Probe("never", log);
            Command timed = race(never.cmd(new Object(), 0, ConflictBehavior.OVERRIDE), waitMs(40));
            Scheduler.schedule(timed);
            long end = System.currentTimeMillis() + 2000;
            while (Scheduler.isScheduled(timed) && System.currentTimeMillis() < end) { Scheduler.execute(); try { Thread.sleep(5); } catch (InterruptedException e) { } }
            check(!Scheduler.isScheduled(timed) && log.contains("never:end:INTERRUPTED"), "timed out the endless command: " + log);
            boolean[] gate = {false};
            Command gated = sequential(waitUntil(() -> gate[0]), instant(() -> log.add("after-gate")));
            Scheduler.schedule(gated); spin(3);
            check(!log.contains("after-gate"), "waiting");
            gate[0] = true; spin(4);
            check(log.contains("after-gate"), "released");
            boolean[] stop = {false};
            Command untilCmd = infinite(() -> {}).until(() -> stop[0]);
            Scheduler.schedule(untilCmd); spin(2);
            check(Scheduler.isScheduled(untilCmd), "infinite until: running");
            stop[0] = true; spin(3);
            check(!Scheduler.isScheduled(untilCmd), "until() ended it");
        }));

        test("ivy: Scheduler.reset() forgets commands WITHOUT calling end() (so OpModes must shut hardware down themselves)", () -> fresh(() -> {
            List<String> log = new ArrayList<>();
            Probe a = new Probe("a", log);
            Scheduler.schedule(a.cmd(new Object(), 0, ConflictBehavior.OVERRIDE)); spin(2);
            Scheduler.reset();
            check(!log.contains("a:end:INTERRUPTED") && !log.contains("a:end:NATURALLY"), "no end() on reset: " + log);
            int before = a.executes; spin(3);
            check(a.executes == before, "and nothing executes afterwards");
        }));

        test("ivy: the robot's own commands use END (never resume a half-finished feed) and CANCEL/OVERRIDE as documented", () -> fresh(() -> {
            Command shoot = RobotCommands.shoot(null, 1, 1800);
            check(shoot.interruptedBehavior() == InterruptedBehavior.END, "shoot ends, never suspends");
            check(shoot.conflictBehavior() == ConflictBehavior.CANCEL && shoot.priority() == 1, "shoot cannot be displaced by an equal-priority command");
            Command intake = RobotCommands.runIntake(null, false);
            check(intake.priority() < shoot.priority(), "intake below shoot");
            Command stop = RobotCommands.stopFlywheel(null);
            check(stop.priority() > 5, "stop outranks the regulator (priority 5)");
            check(RobotCommands.spinUpFlywheel(null, 1800).requirements().isEmpty(), "spin-up owns no resource");
        }));
    }
}
