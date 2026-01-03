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
 *   - calibrationDistancesIn[] / calibrationSpeedsRpm[] (UPDATED 2025-11-15)
 *       • Ordered calibration table (distance inches + RPM) used for interpolation.
 *       • AutoRpmConfig.apply(...) overwrites these arrays every init; provide any
 *         number of points ≥ 2. The controller clamps outside the range and uses
 *         linear interpolation inside.
 *   - smoothingAlpha
 *       • 0–1 exponential smoothing factor (0 disables smoothing).
 *       • AutoRpmConfig.apply(...) should set this to SMOOTH_ALPHA so TeleOp lab
 *         testing matches match-day settings.
 *   - defaultRpm (ADDED 2025-10-31)
 *       • RPM commanded when AutoSpeed is enabled but no AprilTag distance is
 *         available yet. AutoRpmConfig derives this from the farthest calibration point.
 *
 * METHODS
 *   - setCalibrationCurve(...)
 *       • Updates the calibration table; typically called only from AutoRpmConfig.
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
 *   - Provide at least two calibration points so interpolation works correctly.
 */
package org.firstinspires.ftc.teamcode.control;

import java.util.Arrays;

public class LauncherAutoSpeedController {

    // === TUNABLES (RPM + inches) ===
    // CHANGES (2025-10-31): Updated default anchors (65.4 in → 4550 RPM, 114 in → 5000 RPM), default hold RPM,
    //                       and now remember the last vision-calculated RPM so the default only seeds pre-lock behavior.
    // CHANGES (2025-11-15): Replaced near/far anchors with a generic calibration table + clamping interpolation.
    // CHANGES (2026-01-03): Default no-tag RPM now derives from the farthest calibration point via AutoRpmConfig.
    private double[] calibrationDistancesIn = {65.4, 114.0};   // Ordered distance list (in)
    private double[] calibrationSpeedsRpm   = {4550.0, 5000.0}; // RPM paired with each distance entry

    // === MODE / STATE ===
    private boolean autoEnabled   = false; // Tracks if auto mode is currently feeding RPM updates
    private double  defaultRpm    = 4450.0; // RPM to hold before first tag lock; overwritten by AutoRpmConfig
    private Double  lastVisionRpm = null;  // Remembers the last RPM produced from vision (null until first lock)
    private double  lastAutoRpm   = defaultRpm;   // Last commanded RPM (default until a vision lock updates it)

    // === SMOOTHING (0..1). 0 disables smoothing, 0.10–0.30 = gentle smoothing. ===
    private double smoothingAlpha = 0.15;  // Exponential smoothing factor; AutoRpmConfig resets this each apply()

    // -------------------------------------------------------------------------
    // CONFIGURATION
    // -------------------------------------------------------------------------
    /**
     * Configure the calibration table used for interpolation.
     * Arrays must be the same length (≥2). Values are copied and sorted by distance ascending.
     */
    public void setCalibrationCurve(double[] distancesIn, double[] rpmValues) {
        if (distancesIn == null || rpmValues == null) {
            throw new IllegalArgumentException("Calibration arrays cannot be null");
        }
        if (distancesIn.length != rpmValues.length || distancesIn.length < 2) {
            throw new IllegalArgumentException("Calibration arrays must match and contain at least two points");
        }

        double[] distCopy = Arrays.copyOf(distancesIn, distancesIn.length);
        double[] rpmCopy  = Arrays.copyOf(rpmValues, rpmValues.length);

        // Insertion sort so distances ascend while keeping RPM pairs aligned.
        for (int i = 1; i < distCopy.length; i++) {
            double keyDist = distCopy[i];
            double keyRpm  = rpmCopy[i];
            int j = i - 1;
            while (j >= 0 && distCopy[j] > keyDist) {
                distCopy[j + 1] = distCopy[j];
                rpmCopy[j + 1]  = rpmCopy[j];
                j--;
            }
            distCopy[j + 1] = keyDist;
            rpmCopy[j + 1]  = keyRpm;
        }

        this.calibrationDistancesIn = distCopy;
        this.calibrationSpeedsRpm   = rpmCopy;
    }

    /** Set smoothing alpha in [0,1]. 0 = no smoothing. */
    public void setSmoothingAlpha(double alpha) { this.smoothingAlpha = clamp01(alpha); }

    /**
     * Seed the RPM used before a tag lock is acquired.
     * Only resets the commanded RPM when no tag lock has ever been processed.
     */
    public void setDefaultRpm(double defaultRpm) {
        this.defaultRpm = defaultRpm;
        if (lastVisionRpm == null) {
            this.lastAutoRpm = defaultRpm;
        }
    }

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
     *  - Else: maps the distance through the calibration table (clamped + interpolated),
     *          applies optional smoothing, records, and returns lastAutoRpm.
     */
    public double updateWithVision(Double distanceInchesOrNull) {
        if (!autoEnabled) {
            return lastAutoRpm;
        }

        if (distanceInchesOrNull == null) {
            // Tag lost: hold last vision-derived RPM if available, otherwise fall back to default seed.
            lastAutoRpm = (lastVisionRpm != null) ? lastVisionRpm : defaultRpm;
            return lastAutoRpm;
        }

        double dIn = distanceInchesOrNull;
        double rpm = mapDistanceToRpmTable(dIn);

        if (smoothingAlpha > 0.0) {
            rpm = lastAutoRpm + smoothingAlpha * (rpm - lastAutoRpm);
        }

        lastVisionRpm = rpm;
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
    public double[] getCalibrationDistancesIn() { return Arrays.copyOf(calibrationDistancesIn, calibrationDistancesIn.length); }
    public double[] getCalibrationSpeedsRpm()   { return Arrays.copyOf(calibrationSpeedsRpm, calibrationSpeedsRpm.length); }
    public double getSmoothingAlpha() { return smoothingAlpha; }
    public double getDefaultRpm()     { return defaultRpm; }

    // -------------------------------------------------------------------------
    // INTERNALS
    // -------------------------------------------------------------------------
    /** Map distance to RPM using the calibration table with clamping and interpolation. */
    private double mapDistanceToRpmTable(double distanceInches) {
        double[] dist = calibrationDistancesIn;
        double[] rpm  = calibrationSpeedsRpm;
        if (dist.length == 0) {
            return defaultRpm;
        }
        if (distanceInches <= dist[0]) {
            return rpm[0];
        }
        int lastIdx = dist.length - 1;
        if (distanceInches >= dist[lastIdx]) {
            return rpm[lastIdx];
        }

        for (int i = 1; i < dist.length; i++) {
            double nextDist = dist[i];
            if (distanceInches <= nextDist) {
                double prevDist = dist[i - 1];
                double prevRpm  = rpm[i - 1];
                double nextRpm  = rpm[i];
                double span = nextDist - prevDist;
                if (Math.abs(span) < 1e-9) {
                    return nextRpm;
                }
                double t = (distanceInches - prevDist) / span;
                return prevRpm + t * (nextRpm - prevRpm);
            }
        }

        return rpm[lastIdx];
    }

    private static double clamp01(double v) {
        if (v < 0) return 0;
        if (v > 1) return 1;
        return v;
    }
}
