/*
 * FILE: AutoAimTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Provide optional overrides for AutoAimSpeed behavior without editing the
 *     assist class directly. These values default to the shared robot tuning so
 *     TeleOp and Auto stay aligned unless you explicitly diverge them here.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → AutoAim, targeting, and AprilTag alignment)
 *   - MAX_TWIST
 *       • Clamp applied to the twist recommendation returned by AutoAimSpeed.
 *         Defaults to SharedRobotTuning.TURN_TWIST_CAP; raise only if AutoAim
 *         needs to turn faster than BaseAuto helpers.
 *   - RPM_TOLERANCE
 *       • Readiness window used when checking launcher at-speed status. Matches
 *         SharedRobotTuning.RPM_TOLERANCE by default.
 *   - INITIAL_AUTO_DEFAULT_SPEED
 *       • Seed RPM applied before the first tag fix when AutoAim enables. Mirrors
 *         SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED unless overridden.
 *   - AUTO_AIM_SPEED_SCALE (NEW 2025-10-30)
 *       • Fractional drive speed applied to TeleOp translation while AutoAim is
 *         enabled. Lower to tighten alignment, raise toward 1.0 for faster
 *         approaches.
 *   - LONG_SHOT_ENABLED (NEW 2025-11-22)
 *       • Master switch for the alliance-biased long-shot lock window. Set false
 *         to keep the symmetric deadband even when beyond LONG_SHOT_DISTANCE_IN.
 *   - LONG_SHOT_DISTANCE_IN (NEW 2025-11-18)
 *       • Distance threshold (inches) above which AutoAim applies the
 *         alliance-biased lock window for long-range shots.
 *   - INVERT_TWIST (NEW 2026-01-08)
 *       • Optional sign flip applied once where twist feeds the drivebase so
 *         the physical "+rotation" direction matches TagAimController’s
 *         heading error convention (+right / -left).
 */
package org.firstinspires.ftc.teamcode.config;

public final class AutoAimTuning {
    private AutoAimTuning() {}

    // CHANGES (2025-11-22): Added LONG_SHOT_ENABLED master toggle to fall back to symmetric lock windows if needed.
    // CHANGES (2025-11-18): Added LONG_SHOT_DISTANCE_IN for alliance-biased long-range lock windows.
    // CHANGES (2025-10-30): Added AUTO_AIM_SPEED_SCALE for AutoAim drive throttling.
    public static double MAX_TWIST = SharedRobotTuning.TURN_TWIST_CAP;                // Twist clamp applied to aim corrections
    public static double RPM_TOLERANCE = SharedRobotTuning.RPM_TOLERANCE;             // Readiness window while AutoAim checks RPM
    public static double INITIAL_AUTO_DEFAULT_SPEED = SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED; // Seed RPM before first tag fix
    public static double AUTO_AIM_SPEED_SCALE = 0.25;                                 // Translation multiplier (0-1) while AutoAim is active
    public static boolean LONG_SHOT_ENABLED = true;                                    // Master toggle for alliance-biased long-shot lock behavior
    public static double LONG_SHOT_DISTANCE_IN = 90.0;                                // Range cutover (in) for long-shot lock biasing
    // Twist inversion now applies only to AutoAim (manual rotation is never flipped).
    public static boolean INVERT_TWIST = false;                                        // Legacy flag (kept for compatibility)
    public static boolean INVERT_AIM_TWIST = INVERT_TWIST;                             // Flip AutoAim twist only
}
