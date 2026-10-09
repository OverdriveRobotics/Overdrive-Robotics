package org.firstinspires.ftc.teamcode.tuning;

/**
 * Decides whether a velocity reading is expressed in the FIELD or the ROBOT frame by comparing it with the
 * velocity implied by the change in pose (which is always field-frame). Needs heading away from 0 (mod 2pi) and
 * real motion, otherwise the two interpretations coincide and the verdict is UNDETERMINED.
 */
public final class VelocityFrame {
    public enum Verdict { FIELD, ROBOT, UNDETERMINED }

    private int fieldVotes, robotVotes;

    /** @param heading pose heading (rad), (vx,vy) the reported velocity, (dx,dy)/dt the pose-derived velocity. */
    public void add(double heading, double vx, double vy, double dx, double dy, double dt) {
        if (!(dt > 0)) return;
        double ex = dx / dt, ey = dy / dt;
        double speed = Math.hypot(ex, ey);
        if (speed < 3.0 || Math.abs(Math.sin(heading)) < 0.35) return;   // too slow / heading too close to 0 or pi
        double errField = Math.hypot(vx - ex, vy - ey);
        double c = Math.cos(heading), s = Math.sin(heading);
        double rx = vx * c - vy * s, ry = vx * s + vy * c;                // interpret (vx,vy) as robot-frame
        double errRobot = Math.hypot(rx - ex, ry - ey);
        if (errField < 0.5 * errRobot) fieldVotes++;
        else if (errRobot < 0.5 * errField) robotVotes++;
    }

    public int samples() { return fieldVotes + robotVotes; }

    public Verdict verdict() {
        int n = samples();
        if (n < 5) return Verdict.UNDETERMINED;
        if (fieldVotes >= 0.8 * n) return Verdict.FIELD;
        if (robotVotes >= 0.8 * n) return Verdict.ROBOT;
        return Verdict.UNDETERMINED;
    }
}
