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
 *   - BLUE_CALIBRATION_DISTANCES_IN / BLUE_CALIBRATION_RPMS (ADDED 2025-11-15, SPLIT 2026-01-17)
 *       • Ordered arrays of inches + RPM pairs that define the BLUE alliance calibration table.
 *       • Supports any N ≥ 2 entries. LauncherAutoSpeedController interpolates between
 *         neighboring points and clamps outside the table bounds.
 *   - RED_CALIBRATION_DISTANCES_IN / RED_CALIBRATION_RPMS (ADDED 2026-01-17)
 *       • Ordered arrays of inches + RPM pairs that define the RED alliance calibration table.
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
 *   - apply(LauncherAutoSpeedController ctrl, Alliance alliance)
 *       • Copies all tunables into the supplied controller for the active alliance. Call from TeleOp init
 *         and BaseAuto runOpMode() before relying on AutoSpeed.
 *
 * NOTES
 *   - AutoAimSpeed and BaseAuto both read the controller after apply(), so this
 *     file is the single source of truth for curve anchors.
 *   - Provide at least two calibration points; values do not need to be evenly spaced.
 *   - Launcher.RPM_MIN/RPM_MAX still clamp the final command; adjust those in
 *     subsystems/Launcher.java when hardware changes require broader limits.
 *
 * CHANGES (2026-01-17): Split AutoRPM calibration tables by alliance and
 *                       applied the selection through AutoRpmConfig.apply(...).
 */
package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.control.LauncherAutoSpeedController;

public final class AutoRpmConfig {
    private AutoRpmConfig() {}

    // --- Tunables shared by TeleOp & Auto ---
    // CHANGES (2025-11-15): Replaced fixed near/far anchors with a full calibration table (default 6 points, 35–100 in).
    // CHANGES (2026-01-03): Default no-tag RPM now derives from the farthest calibration point.
    // CHANGES (2026-01-17): Split AutoRPM calibration tables by alliance (RED/BLUE) with identical defaults.
    public static double[] BLUE_CALIBRATION_DISTANCES_IN = {
            48.0,
            55.0,
            65.0,
            78.0,
            88.0,
            115.0,
            120.0,
            130.0,
            140.0
    }; // Blue alliance inches for the calibration table (must align with BLUE_CALIBRATION_RPMS)
    public static double[] BLUE_CALIBRATION_RPMS = {
            2350.0,
            2550.0,
            2550.0,
            2700.0,
            2900.0,
            3800.0,
            3900.0,
            4050.0,
            4300.0
    }; // Blue alliance RPM values paired with BLUE_CALIBRATION_DISTANCES_IN entries
    public static double[] RED_CALIBRATION_DISTANCES_IN = {
            48.0,
            55.0,
            65.0,
            78.0,
            88.0,
            115.0,
            120.0,
            130.0,
            140.0
    }; // Red alliance inches for the calibration table (must align with RED_CALIBRATION_RPMS)
    public static double[] RED_CALIBRATION_RPMS = {
            2350.0,
            2550.0,
            2550.0,
            2700.0,
            2900.0,
            3800.0,
            3900.0,
            4050.0,
            4300.0
    }; // Red alliance RPM values paired with RED_CALIBRATION_DISTANCES_IN entries
    public static double SMOOTH_ALPHA      = 0.15;  // Exponential smoothing factor applied after every apply()

    /** Apply standard params to a controller for the requested alliance. Safe to call repeatedly. */
    public static void apply(LauncherAutoSpeedController ctrl, Alliance alliance) {
        if (ctrl == null) return;
        ctrl.setDefaultRpm(resolveNoTagRpm(alliance));
        ctrl.setCalibrationCurve(resolveDistances(alliance), resolveRpms(alliance));
        ctrl.setSmoothingAlpha(SMOOTH_ALPHA);
    }

    /** RPM to hold while AutoSpeed runs without a tag lock (uses farthest calibration point). */
    public static double resolveNoTagRpm(Alliance alliance) {
        double[] distances = resolveDistances(alliance);
        double[] rpms = resolveRpms(alliance);
        if (distances == null || rpms == null) return 0.0;
        if (distances.length == 0 || rpms.length == 0) return 0.0;
        int maxIndex = 0;
        double maxDistance = distances[0];
        int count = Math.min(distances.length, rpms.length);
        for (int i = 1; i < count; i++) {
            if (distances[i] > maxDistance) {
                maxDistance = distances[i];
                maxIndex = i;
            }
        }
        return rpms[maxIndex];
    }

    private static double[] resolveDistances(Alliance alliance) {
        return (alliance == Alliance.RED) ? RED_CALIBRATION_DISTANCES_IN : BLUE_CALIBRATION_DISTANCES_IN;
    }

    private static double[] resolveRpms(Alliance alliance) {
        return (alliance == Alliance.RED) ? RED_CALIBRATION_RPMS : BLUE_CALIBRATION_RPMS;
    }
}
