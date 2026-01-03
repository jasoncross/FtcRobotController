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
 *   - DEFAULT_NO_TAG_RPM (ADDED 2025-10-31, REMOVED 2026-01-03)
 *       • RPM commanded whenever AutoSpeed is active but no AprilTag is locked yet.
 *         Now derived from the farthest calibration point instead of a manual value.
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
    // CHANGES (2026-01-03): Default no-tag RPM now derives from the farthest calibration point.
    public static double[] CALIBRATION_DISTANCES_IN = {
            48.0,
            55.0,
            65.0,
            78.0,
            88.0,
            100.0,
            120.0,
            130.0
    }; // Inches for the calibration table (must align with CALIBRATION_RPMS)
    public static double[] CALIBRATION_RPMS = {
            2750.0,
            2600.0,
            2650.0,
            2800.0,
            3000.0,
            3200.0,
            3900.0,
            4250.0
    }; // RPM values paired with CALIBRATION_DISTANCES_IN entries
    public static double SMOOTH_ALPHA      = 0.15;  // Exponential smoothing factor applied after every apply()

    /** Apply standard params to a controller. Safe to call repeatedly. */
    public static void apply(LauncherAutoSpeedController ctrl) {
        if (ctrl == null) return;
        ctrl.setDefaultRpm(resolveNoTagRpm());
        ctrl.setCalibrationCurve(CALIBRATION_DISTANCES_IN, CALIBRATION_RPMS);
        ctrl.setSmoothingAlpha(SMOOTH_ALPHA);
    }

    /** RPM to hold while AutoSpeed runs without a tag lock (uses farthest calibration point). */
    public static double resolveNoTagRpm() {
        if (CALIBRATION_DISTANCES_IN == null || CALIBRATION_RPMS == null) return 0.0;
        if (CALIBRATION_DISTANCES_IN.length == 0 || CALIBRATION_RPMS.length == 0) return 0.0;
        int maxIndex = 0;
        double maxDistance = CALIBRATION_DISTANCES_IN[0];
        int count = Math.min(CALIBRATION_DISTANCES_IN.length, CALIBRATION_RPMS.length);
        for (int i = 1; i < count; i++) {
            if (CALIBRATION_DISTANCES_IN[i] > maxDistance) {
                maxDistance = CALIBRATION_DISTANCES_IN[i];
                maxIndex = i;
            }
        }
        return CALIBRATION_RPMS[maxIndex];
    }
}
