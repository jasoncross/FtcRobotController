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
 *   - CALIBRATION_DISTANCES_IN / CALIBRATION_RPMS (ADDED 2025-11-15)
 *       • Ordered arrays of inches + RPM pairs that define the entire calibration table.
 *       • Supports any N ≥ 2 entries. LauncherAutoSpeedController interpolates between
 *         neighboring points and clamps outside the table bounds.
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
 *   - Provide at least two calibration points; values do not need to be evenly spaced.
 *   - Launcher.RPM_MIN/RPM_MAX still clamp the final command; adjust those in
 *     subsystems/Launcher.java when hardware changes require broader limits.
 */
package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.control.LauncherAutoSpeedController;

public final class AutoRpmConfig {
    private AutoRpmConfig() {}

    // --- Tunables shared by TeleOp & Auto ---
    // CHANGES (2025-11-15): Replaced fixed near/far anchors with a full calibration table (default 6 points, 35–100 in).
    public static double[] CALIBRATION_DISTANCES_IN = {
            35.0,
            37.0,
            60.0,
            67.0,
            82.0,
            100.0
    }; // Inches for the calibration table (must align with CALIBRATION_RPMS)
    public static double[] CALIBRATION_RPMS = {
            2600.0,
            2500.0,
            2550.0,
            2750.0,
            3050.0,
            3800.0
    }; // RPM values paired with CALIBRATION_DISTANCES_IN entries
    public static double SMOOTH_ALPHA      = 0.15;  // Exponential smoothing factor applied after every apply()
    public static double DEFAULT_NO_TAG_RPM = 2500.0; // RPM to hold while AutoSpeed runs without a tag lock

    /** Apply standard params to a controller. Safe to call repeatedly. */
    public static void apply(LauncherAutoSpeedController ctrl) {
        if (ctrl == null) return;
        ctrl.setDefaultRpm(DEFAULT_NO_TAG_RPM);
        ctrl.setCalibrationCurve(CALIBRATION_DISTANCES_IN, CALIBRATION_RPMS);
        ctrl.setSmoothingAlpha(SMOOTH_ALPHA);
    }
}
