/*
 * FILE: AutoRpmConfig.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Store the authoritative distance→RPM anchor points and smoothing factor
 *     for LauncherAutoSpeedController so TeleOp and Autonomous read the exact
 *     same curve each time they initialize.
 *   - Provide a single apply(...) helper invoked from BaseAuto and
 *     TeleOpAllianceBase to copy the values into the controller at runtime.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Launcher speed & flywheel control)
 *   - NEAR_DIST_IN / NEAR_RPM
 *       • Inches + RPM pair representing the calibrated "close" shot.
 *       • These override the controller’s defaults on every apply(); adjust both
 *         together after recalibrating VisionAprilTag range scaling.
 *   - FAR_DIST_IN / FAR_RPM
 *       • Anchor point for far-field volleys.
 *       • Move the distance if autos park farther away; tweak RPM to hold arc
 *         consistency. This still respects Launcher.RPM_MAX clamp downstream.
 *   - SMOOTH_ALPHA
 *       • Exponential smoothing constant (0–1) applied to RPM updates.
 *       • Overrides LauncherAutoSpeedController.smoothingAlpha; match the value
 *         here to what TunableDirectory recommends so TeleOp lab tests mirror
 *         match play.
 *   - DEFAULT_NO_TAG_RPM (ADDED 2025-10-31)
 *       • RPM commanded whenever AutoSpeed is active but no AprilTag is locked yet.
 *         Keeps the flywheel spooled until distance data arrives.
 *
 * METHODS
 *   - apply(LauncherAutoSpeedController ctrl)
 *       • Copies all tunables into the supplied controller. Call from TeleOp init
 *         and BaseAuto runOpMode() before relying on AutoSpeed.
 *
 * NOTES
 *   - AutoAimSpeed and BaseAuto both read the controller after apply(), so this
 *     file is the single source of truth for curve anchors.
 *   - Launcher.RPM_MIN/RPM_MAX still clamp the final command; adjust those in
 *     subsystems/Launcher.java when hardware changes require broader limits.
 */
package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.control.LauncherAutoSpeedController;

public final class AutoRpmConfig {
    private AutoRpmConfig() {}

    // --- Tunables shared by TeleOp & Auto ---
    // CHANGES (2025-10-31): Updated anchor defaults (65.4 in → 4550 RPM, 114 in → 5000 RPM) and added DEFAULT_NO_TAG_RPM.
    public static double NEAR_DIST_IN      = 43;  // Authoritative near-shot distance (inches); overrides controller defaults
    public static double NEAR_RPM          = 4100.0; // RPM paired with NEAR_DIST_IN; adjust together after re-scaling range
    public static double FAR_DIST_IN       = 66.0; // Far-shot anchor distance; align with autonomous standoff distances
    public static double FAR_RPM           = 4500.0; // RPM for FAR_DIST_IN; still clamped by Launcher.RPM_MAX
    public static double SMOOTH_ALPHA      = 0.15;  // Exponential smoothing factor applied after every apply()
    public static double DEFAULT_NO_TAG_RPM = 4450.0; // RPM to hold while AutoSpeed runs without a tag lock

    /** Apply standard params to a controller. Safe to call repeatedly. */
    public static void apply(LauncherAutoSpeedController ctrl) {
        if (ctrl == null) return;
        ctrl.setDefaultRpm(DEFAULT_NO_TAG_RPM);
        ctrl.setParams(NEAR_DIST_IN, NEAR_RPM, FAR_DIST_IN, FAR_RPM);
        ctrl.setSmoothingAlpha(SMOOTH_ALPHA);
    }
}
