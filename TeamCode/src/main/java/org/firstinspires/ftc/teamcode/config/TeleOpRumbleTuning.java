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
    public static boolean AIM_RUMBLE_ENABLED   = true;  // Master enable for manual aim rumble feedback
    public static double  AIM_THRESHOLD_DEG    = 2.5;   // Heading error (deg) at which rumble begins
    public static double  AIM_STRENGTH_MIN     = 0.10;  // Minimum rumble strength when just outside threshold
    public static double  AIM_STRENGTH_MAX     = 0.65;  // Maximum rumble strength at large heading error
    public static int     AIM_PULSE_MIN_MS     = 120;   // Shortest rumble pulse length (ms) at low error
    public static int     AIM_PULSE_MAX_MS     = 200;   // Longest rumble pulse length (ms) at high error
    public static int     AIM_COOLDOWN_MIN_MS  = 120;   // Minimum cooldown between pulses (ms)
    public static int     AIM_COOLDOWN_MAX_MS  = 350;   // Maximum cooldown between pulses (ms)

    // Toggle confirmation pulses
    public static double TOGGLE_STRENGTH = 0.8; // Strength of each rumble burst for toggle confirmations
    public static int    TOGGLE_STEP_MS  = 120; // Duration (ms) of each rumble step in the toggle pattern
    public static int    TOGGLE_GAP_MS   = 80;  // Gap (ms) between toggle rumble steps
}
