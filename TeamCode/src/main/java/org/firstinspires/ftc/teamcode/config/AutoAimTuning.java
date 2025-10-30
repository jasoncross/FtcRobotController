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
 */
package org.firstinspires.ftc.teamcode.config;

public final class AutoAimTuning {
    private AutoAimTuning() {}

    // CHANGES (2025-10-30): Added AUTO_AIM_SPEED_SCALE for AutoAim drive throttling.
    public static double MAX_TWIST = SharedRobotTuning.TURN_TWIST_CAP;
    public static double RPM_TOLERANCE = SharedRobotTuning.RPM_TOLERANCE;
    public static double INITIAL_AUTO_DEFAULT_SPEED = SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED;
    public static double AUTO_AIM_SPEED_SCALE = 0.25; // Translation multiplier (0-1) while AutoAim is active
}
