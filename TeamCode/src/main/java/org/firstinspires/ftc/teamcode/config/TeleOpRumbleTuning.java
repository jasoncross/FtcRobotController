/*
 * FILE: TeleOpRumbleTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Centralize the driver-feedback rumble envelopes used in TeleOp so strength,
 *     pulse length, and toggle confirmation cues are easy to adjust between
 *     events without editing TeleOpAllianceBase.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Driver feedback)
 *   - AIM_RUMBLE_ENABLED
 *       • Master enable for manual aim rumble when AutoAim is off.
 *   - AIM_THRESHOLD_DEG / AIM_STRENGTH_MIN/MAX / AIM_PULSE_MIN/MAX / AIM_COOLDOWN_MIN/MAX
 *       • Shape the rumble response window while aiming manually.
 *   - TOGGLE_STRENGTH / TOGGLE_STEP_MS / TOGGLE_GAP_MS
 *       • Haptic pattern used when toggling assists (AutoAim, AutoSpeed, etc.).
 */
package org.firstinspires.ftc.teamcode.config;

public final class TeleOpRumbleTuning {
    private TeleOpRumbleTuning() {}

    // Manual aim rumble defaults
    public static boolean AIM_RUMBLE_ENABLED   = true;
    public static double  AIM_THRESHOLD_DEG    = 2.5;
    public static double  AIM_STRENGTH_MIN     = 0.10;
    public static double  AIM_STRENGTH_MAX     = 0.65;
    public static int     AIM_PULSE_MIN_MS     = 120;
    public static int     AIM_PULSE_MAX_MS     = 200;
    public static int     AIM_COOLDOWN_MIN_MS  = 120;
    public static int     AIM_COOLDOWN_MAX_MS  = 350;

    // Toggle confirmation pulses
    public static double TOGGLE_STRENGTH = 0.8;
    public static int    TOGGLE_STEP_MS  = 120;
    public static int    TOGGLE_GAP_MS   = 80;
}
