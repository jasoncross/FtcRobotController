/*
 * FILE: LauncherAutoSpeedController.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/control/
 *
 * PURPOSE
 *   - Convert AprilTag-measured distance (inches) into a launcher RPM target so
 *     AutoAimSpeed, BaseAuto, and TeleOpAllianceBase share the same AutoSpeed
 *     behavior.
 *   - Smoothly transition between automatically computed RPM and manual driver
 *     overrides by remembering the last auto value.
 *   - Offer optional exponential smoothing for vision updates that flicker.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Launcher speed & flywheel control)
 *   - nearDistanceIn / nearSpeedRpm & farDistanceIn / farSpeedRpm
 *       • Anchor points for the linear distance→RPM mapping.
 *       • AutoRpmConfig.apply(...) overwrites these every init; edit that file for
 *         authoritative values. These fields only matter when running isolated
 *         tests without apply().
 *   - smoothingAlpha
 *       • 0–1 exponential smoothing factor (0 disables smoothing).
 *       • AutoRpmConfig.apply(...) should set this to SMOOTH_ALPHA so TeleOp lab
 *         testing matches match-day settings.
 *
 * METHODS
 *   - setParams(...)
 *       • Updates the anchor points; typically called only from AutoRpmConfig.
 *   - setSmoothingAlpha(...)
 *       • Sets the smoothing factor, clamped to [0,1].
 *   - setAutoEnabled()/isAutoEnabled()
 *       • Toggle and query whether automatic control is active.
 *   - updateWithVision(...)
 *       • Map distance to RPM, optionally smooth, and remember the last auto
 *         value so AutoAimSpeed can query hold().
 *   - onManualOverride(...)
 *       • Sync lastAutoRpm with a manual RPM so toggling feels seamless.
 *   - hold() + getters
 *       • Expose state for telemetry dashboards.
 *
 * NOTES
 *   - BaseAuto and AutoAimSpeed both call updateWithVision() every loop, so keep
 *     math light to avoid frame drops.
 *   - The mapping extrapolates beyond both anchors; ensure the near/far points
 *     bracket your expected shooting distances to avoid extreme RPM requests.
 */
package org.firstinspires.ftc.teamcode.control;

public class LauncherAutoSpeedController {

    // === TUNABLES (RPM + inches) ===
    private double nearDistanceIn = 24.0;  // Near anchor distance (in); AutoRpmConfig overwrites on init
    private double nearSpeedRpm   = 1000.0; // RPM at near anchor; matches AutoRpmConfig.NEAR_RPM unless testing locally
    private double farDistanceIn  = 120.0; // Far anchor distance (in); overwritten by AutoRpmConfig
    private double farSpeedRpm    = 4500.0; // RPM at far anchor; still clamped by Launcher.RPM_MAX

    // === MODE / STATE ===
    private boolean autoEnabled   = false; // Tracks if auto mode is currently feeding RPM updates
    private double  lastAutoRpm   = 0.0;   // Last computed auto RPM (also used when tag data drops)

    // === SMOOTHING (0..1). 0 disables smoothing, 0.10–0.30 = gentle smoothing. ===
    private double smoothingAlpha = 0.15;  // Exponential smoothing factor; AutoRpmConfig resets this each apply()

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
