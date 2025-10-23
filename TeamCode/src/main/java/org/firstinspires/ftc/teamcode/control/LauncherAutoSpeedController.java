// ============================================================================
// FILE:           LauncherAutoSpeedController.java
// LOCATION:       org/firstinspires/ftc/teamcode/control/
// PURPOSE:        Compute automatic flywheel TARGET SPEED (in RPM) from robot
//                 distance (inches) to the GOAL AprilTag using four tunables:
//                   - nearDistanceIn (inches)
//                   - nearSpeedRpm   (RPM)
//                   - farDistanceIn  (inches)
//                   - farSpeedRpm    (RPM)
//                 Behavior:
//                   - LINEAR mapping with EXTRAPOLATION beyond near/far bounds.
//                   - Holds last auto-computed RPM when tag is not visible.
//                   - Optional smoothing to reduce abrupt RPM changes.
//                   - Clean handoff between manual and auto modes.
// NOTES:
//   • Units:
//       - Distance: INCHES (convert from meters upstream if needed).
//       - Speed:    RPM (matches Launcher closed-loop RPM control).
//   • Extrapolation:
//       - If d < nearDistanceIn or d > farDistanceIn, RPM is computed by
//         extending the same line (no clamping). 12", 130" both work.
//   • Initial Team Defaults (update via OpModes):
//       - 24.0 in → 1000 RPM
//       - 120.0 in → 4500 RPM
//   • Curve Choice:
//       - Start LINEAR. If testing shows systematic error, we can swap the
//         mapping to piecewise/quadratic here without changing callers.
// METHODS (PUBLIC):
//   - setParams(nearDistIn, nearRpm, farDistIn, farRpm)
//   - setSmoothingAlpha(alpha)          // 0.0 = no smoothing (default 0.15)
//   - setAutoEnabled(enabled)
//   - isAutoEnabled()
//   - updateWithVision(distanceInchesOrNull)  // returns target RPM
//   - onManualOverride(currentManualRpm)      // seeds last RPM & disables auto
//   - hold()                                  // returns last auto RPM
//   - getters for tunables and last RPM
// AUTHOR:         Indianola Robotics – 2025 Season (DECODE)
// LAST UPDATED:   2025-10-22
// ============================================================================
package org.firstinspires.ftc.teamcode.control;

public class LauncherAutoSpeedController {

    // === TUNABLES (RPM + inches) ===
    private double nearDistanceIn = 24.0;
    private double nearSpeedRpm   = 1000.0;
    private double farDistanceIn  = 120.0;
    private double farSpeedRpm    = 4500.0;

    // === MODE / STATE ===
    private boolean autoEnabled   = false;
    private double  lastAutoRpm   = 0.0;

    // === SMOOTHING (0..1). 0 disables smoothing, 0.10–0.30 = gentle smoothing. ===
    private double smoothingAlpha = 0.15;

    // -------------------------------------------------------------------------
    // CONFIGURATION
    // -------------------------------------------------------------------------
    /** Configure the four key tunables (inches + RPM). */
    public void setParams(double nearDistanceIn, double nearSpeedRpm,
                          double farDistanceIn,  double farSpeedRpm) {
        this.nearDistanceIn = nearDistanceIn;
        this.nearSpeedRpm   = nearSpeedRpm;
        this.farDistanceIn  = farDistanceIn;
        this.farSpeedRpm    = farSpeedRpm;

        // Normalize ordering so nearDistance <= farDistance (swap paired values if needed)
        if (this.farDistanceIn < this.nearDistanceIn) {
            double td = this.farDistanceIn; this.farDistanceIn = this.nearDistanceIn; this.nearDistanceIn = td;
            double ts = this.farSpeedRpm;   this.farSpeedRpm   = this.nearSpeedRpm;  this.nearSpeedRpm   = ts;
        }
    }

    /** Set smoothing alpha in [0,1]. 0 = no smoothing. */
    public void setSmoothingAlpha(double alpha) { this.smoothingAlpha = clamp01(alpha); }

    /** Enable/disable automatic RPM mode. */
    public void setAutoEnabled(boolean enabled) { this.autoEnabled = enabled; }

    /** @return true if auto mode is enabled. */
    public boolean isAutoEnabled() { return autoEnabled; }

    // -------------------------------------------------------------------------
    // RUNTIME
    // -------------------------------------------------------------------------
    /**
     * Update with latest distance to the GOAL AprilTag.
     * @param distanceInchesOrNull  Distance in INCHES, or null if tag not visible.
     * @return target RPM to command to the launcher.
     *
     * Behavior:
     *  - If auto disabled: returns last auto RPM (no change).
     *  - If distance null: holds last auto RPM (tag lost).
     *  - Else: computes LINEAR mapping with EXTRAPOLATION, applies optional smoothing,
     *          records and returns lastAutoRpm.
     */
    public double updateWithVision(Double distanceInchesOrNull) {
        if (!autoEnabled) {
            return lastAutoRpm;
        }

        if (distanceInchesOrNull == null) {
            // Tag lost: hold last
            return lastAutoRpm;
        }

        double dIn = distanceInchesOrNull;
        double rpm = mapDistanceToRpmLinearExtrapolate(dIn);

        if (smoothingAlpha > 0.0) {
            rpm = lastAutoRpm + smoothingAlpha * (rpm - lastAutoRpm);
        }

        lastAutoRpm = rpm;
        return lastAutoRpm;
    }

    /** Call when switching to manual control to keep transitions seamless. */
    public void onManualOverride(double currentManualRpm) {
        this.lastAutoRpm = currentManualRpm;
        this.autoEnabled = false;
    }

    /** Return last computed auto RPM without updating. */
    public double hold() { return lastAutoRpm; }

    // -------------------------------------------------------------------------
    // TELEMETRY GETTERS
    // -------------------------------------------------------------------------
    public double getLastAutoRpm()    { return lastAutoRpm; }
    public double getNearDistanceIn() { return nearDistanceIn; }
    public double getNearSpeedRpm()   { return nearSpeedRpm; }
    public double getFarDistanceIn()  { return farDistanceIn; }
    public double getFarSpeedRpm()    { return farSpeedRpm; }
    public double getSmoothingAlpha() { return smoothingAlpha; }

    // -------------------------------------------------------------------------
    // INTERNALS
    // -------------------------------------------------------------------------
    /**
     * LINEAR interpolation with EXTRAPOLATION.
     *   slope = (farRPM - nearRPM) / (farDist - nearDist)
     *   rpm(d) = nearRPM + slope * (d - nearDist)
     * Continues past both anchors (no clamping).
     */
    private double mapDistanceToRpmLinearExtrapolate(double distanceInches) {
        double span = (farDistanceIn - nearDistanceIn);
        if (Math.abs(span) < 1e-9) {
            // Degenerate: distances equal; return near RPM.
            return nearSpeedRpm;
        }
        double slope = (farSpeedRpm - nearSpeedRpm) / span;
        return nearSpeedRpm + slope * (distanceInches - nearDistanceIn);
    }

    private static double clamp01(double v) {
        if (v < 0) return 0;
        if (v > 1) return 1;
        return v;
    }
}
