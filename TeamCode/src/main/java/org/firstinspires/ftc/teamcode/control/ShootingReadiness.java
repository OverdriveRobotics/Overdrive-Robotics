package org.firstinspires.ftc.teamcode.control;

/**
 * Pure shooting-readiness evaluation. Everything is computed from the supplied live inputs on every call;
 * nothing is remembered between calls, so readiness can never be "stuck" true.
 * The first failing condition is returned as the human-readable reason.
 */
public final class ShootingReadiness {
    private ShootingReadiness() {}

    public static final class Inputs {
        // localization
        public boolean requireLocalization;
        public boolean poseFinite;
        public double poseAgeMs = Double.POSITIVE_INFINITY;
        public double maxPoseAgeMs = TurretConfig.MAX_POSE_AGE_MS;
        // shooting region (axis-aligned rectangle) and motion
        public boolean requireZone;
        public double x, y, minX, maxX, minY, maxY;
        public double speed, maxSpeed = Double.POSITIVE_INFINITY;
        // turret
        public boolean requireAim;
        public boolean targetValid;
        public boolean turretSensorValid;
        public boolean turretReachable = true;
        public double turretErrorRad = Double.NaN, turretVelRadS = Double.NaN;
        public double alignTolRad = TurretConfig.ALIGN_TOLERANCE_RAD, alignVelTolRadS = TurretConfig.ALIGN_VELOCITY_TOL_RAD_S;
        public String turretFault;
        // flywheel
        public double flywheelTarget;
        /** True only if the regulator has seen the velocity inside tolerance for the full stability window, recently. */
        public boolean flywheelStableFresh;
        public String flywheelFault;
        // feed mechanism
        public boolean feedPermits = true;
        public String feedBlockReason;
    }

    public static final class Result {
        public final boolean ready;
        public final String reason;
        Result(boolean ready, String reason) { this.ready = ready; this.reason = reason; }
        @Override public String toString() { return ready ? "READY" : reason; }
    }

    public static Result evaluate(Inputs in) {
        if (in.flywheelFault != null) return no("flywheel fault: " + in.flywheelFault);
        if (in.requireLocalization || in.requireZone || in.requireAim) {
            if (!in.poseFinite) return no("localization invalid");
            if (!(in.poseAgeMs <= in.maxPoseAgeMs)) return no("localization stale (" + (long) in.poseAgeMs + " ms)");
        }
        if (in.requireZone) {
            if (!(in.x >= in.minX && in.x <= in.maxX && in.y >= in.minY && in.y <= in.maxY))
                return no("outside shooting zone");
        }
        if (in.requireZone || in.requireAim) {
            if (!(in.speed <= in.maxSpeed)) return no("robot too fast (" + String.format("%.1f", in.speed) + ")");
        }
        if (in.requireAim) {
            if (in.turretFault != null) return no("turret fault: " + in.turretFault);
            if (!in.targetValid) return no("selected target invalid");
            if (!in.turretSensorValid) return no("turret sensor invalid");
            if (!in.turretReachable) return no("target outside turret range");
            if (!TurretUtil.isAligned(in.turretErrorRad, in.turretVelRadS, in.alignTolRad, in.alignVelTolRadS))
                return no("turret not aligned (err " + String.format("%.1f", Math.toDegrees(in.turretErrorRad)) + " deg)");
        }
        if (!(in.flywheelTarget > 0)) return no("flywheel target not set");
        if (!in.flywheelStableFresh) return no("flywheel not at stable velocity");
        if (!in.feedPermits) return no(in.feedBlockReason != null ? in.feedBlockReason : "feed mechanism busy");
        return new Result(true, "ready");
    }

    private static Result no(String why) { return new Result(false, why); }
}
