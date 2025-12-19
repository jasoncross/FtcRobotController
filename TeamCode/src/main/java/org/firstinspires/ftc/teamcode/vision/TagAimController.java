package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.config.TagAimTuning;
import org.firstinspires.ftc.teamcode.config.AutoAimTuning;
import org.firstinspires.ftc.teamcode.vision.VisionTargetProvider;

/*
 * FILE: TagAimController.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/
 *
 * PURPOSE
 *   - Translate AprilTag bearing error into a twist command shared by TeleOp
 *     aim assist and Autonomous helpers.
 *   - Provide convenience helpers for telemetry (heading/range).
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → AutoAim)
 *   - kP / kD
 *       • Proportional and derivative gains. Increase kP toward 0.025 for faster
 *         alignment; add kD (~0.004) if oscillations appear.
 *       • Coordinate with SharedRobotTuning.TURN_TWIST_CAP so PD output has room
 *         to act.
 *   - Clamp (±0.6) and deadband (±1.5°)
 *       • Limit twist strength and stop hunting. Expand clamp toward ±0.8 only if
 *         SharedRobotTuning.TURN_TWIST_CAP also increases.
 *
 * METHODS
 *   - setGains(kP, kD)
 *       • Update PD terms to match new tuning.
 *   - turnPower()
 *       • Return twist suggestion in [-0.6, 0.6] based on heading error from the active provider.
 *   - headingDeg() / distanceMeters()
 *       • Telemetry helpers returning NaN when no tag is present via the provider.
 *
 * CHANGES (2025-11-18): Added asymmetric deadband support so callers can bias
 *                       the lock window for long-distance shots.
 * CHANGES (2025-12-11): Routed heading + availability through VisionTargetProvider
 *                       to decouple aim from legacy webcam implementations while
 *                       preserving PD tuning and deadband behavior.
 * CHANGES (2025-12-16): Auto now consumes smoothed goal visibility + held tx
 *                       samples from LimelightTargetProvider so brief dropouts
 *                       do not flip scans; TeleOp behavior remains unchanged.
 * CHANGES (2025-12-21): Aligned turn gating with goalAimValid so twist output
 *                       only engages when a finite heading sample exists,
 *                       matching the new goalDetected vs. aim-valid split.
 */
public class TagAimController {
    private VisionTargetProvider provider;     // Source for target visibility + heading error
    private double kP = TagAimTuning.KP;        // Proportional gain (twist per degree); align with TunableDirectory AutoAim table
    private double kD = TagAimTuning.KD;        // Derivative gain to damp overshoot; increase slightly when oscillations appear
    private double clampAbs = TagAimTuning.CLAMP_ABS;      // Twist clamp magnitude (±clampAbs)
    private double deadbandMinDeg = -Math.abs(TagAimTuning.DEADBAND_DEG); // Deadband lower bound (can bias around zero)
    private double deadbandMaxDeg =  Math.abs(TagAimTuning.DEADBAND_DEG); // Deadband upper bound (can bias around zero)
    private double lastErrorDeg = 0.0; // Stored from previous frame for D term

    public TagAimController() {
        this(null);
    }

    public TagAimController(VisionTargetProvider provider) {
        this.provider = provider;
    }

    public void setProvider(VisionTargetProvider provider) {
        this.provider = provider;
    }

    public void setGains(double kP, double kD) { this.kP = kP; this.kD = kD; }
    public void setClampAndDeadband(double clampAbs, double deadbandDeg) {
        this.clampAbs = clampAbs;
        setDeadbandWindow(-Math.abs(deadbandDeg), Math.abs(deadbandDeg));
    }

    public void setDeadbandWindow(double minDeg, double maxDeg) {
        deadbandMinDeg = Math.min(minDeg, maxDeg);
        deadbandMaxDeg = Math.max(minDeg, maxDeg);
    }

    public void resetDeadband() {
        setDeadbandWindow(-Math.abs(TagAimTuning.DEADBAND_DEG), Math.abs(TagAimTuning.DEADBAND_DEG));
    }

    /** Returns twist power [-1..1] to align robot to the tag; 0 when target is missing or within deadband. */
    public double turnPower() {
        return turnPowerWithProvider(provider);
    }

    /**
     * Legacy signature to keep existing TeleOp/Auto code compiling while wiring through the new provider
     * abstraction. When no provider is set, the detection is wrapped temporarily to supply heading error.
     */
    public double turnPower(AprilTagDetection det) {
        VisionTargetProvider vision = provider;
        if (vision == null) {
            vision = wrapDetection(det);
        }
        return turnPowerWithProvider(vision);
    }

    public double headingDeg() {
        return provider != null && provider.isGoalAimValid() ? provider.getHeadingErrorDeg() : Double.NaN;
    }

    public double distanceMeters() {
        return provider != null && provider.isGoalVisibleSmoothed() ? provider.getDistanceMeters() : Double.NaN;
    }

    /**
     * Ensure positive twist means “rotate right/clockwise (nose toward +X, decreasing field heading)” at the drivebase.
     * Apply exactly once when handing rotation to the drivetrain to align drive hardware with aim error convention.
     */
    public static double applyDriveTwistSign(double twist) {
        boolean invert = false;
        try {
            invert = AutoAimTuning.INVERT_AIM_TWIST;
        } catch (Throwable ignored) {
            try { invert = AutoAimTuning.INVERT_TWIST; } catch (Throwable ignoredToo) {}
        }
        return invert ? -twist : twist;
    }

    private double turnPowerWithProvider(VisionTargetProvider vision) {
        if (vision == null || !vision.isGoalAimValid()) {
            // Keep derivative sane after target loss so the first frame back uses fresh error
            lastErrorDeg = 0.0;
            return 0.0;
        }
        Double smoothed = vision.getSmoothedHeadingErrorDeg();
        if (smoothed == null || Double.isNaN(smoothed)) {
            lastErrorDeg = 0.0;
            return 0.0;
        }
        double errDeg = smoothed;   // + right, - left
        double deriv  = errDeg - lastErrorDeg; // simple D term (frame-to-frame)
        lastErrorDeg  = errDeg;

        double power = (kP * errDeg) + (kD * deriv);

        // Clamp twist (sane max while still letting you translate at full speed)
        double clamp = Math.abs(clampAbs);
        if (power > clamp) power = clamp;
        if (power < -clamp) power = -clamp;

        // Deadband to stop hunting
        if (errDeg > deadbandMinDeg && errDeg < deadbandMaxDeg) power = 0.0;

        return power;
    }

    private VisionTargetProvider wrapDetection(AprilTagDetection det) {
        if (det == null) {
            return null;
        }
        return new VisionTargetProvider() {
            @Override
            public boolean hasTarget() {
                return true;
            }

            @Override
            public boolean hasGoalTarget() { return hasTarget(); }

            @Override
            public boolean hasAnyTarget() { return hasTarget(); }

            @Override
            public int getBestVisibleTagId() { return det.id; }

            @Override
            public double getHeadingErrorDeg() {
                return det.ftcPose.bearing;
            }

            @Override
            public double getDistanceMeters() {
                return det.ftcPose.range;
            }
        };
    }
}
