/*
 * FILE: TagAimTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Collect PD gains, clamp, and deadband used by TagAimController so vision
 *     alignment tweaks stay outside the logic class and mirror the Tunable
 *     Directory.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → AutoAim, targeting, and AprilTag alignment)
 *   - KP / KD
 *       • Proportional and derivative gains converting AprilTag bearing error to
 *         twist power. Increase KP for faster alignment; add KD to calm overshoot.
 *   - CLAMP_ABS
 *       • Absolute value of the twist clamp (±CLAMP_ABS). Keep ≥ AutoAimTuning
 *         MAX_TWIST so AutoAimSpeed can honor its limits without double-clamping.
 *   - DEADBAND_DEG
 *       • Heading window treated as “close enough” to stop jittering around zero.
 */
package org.firstinspires.ftc.teamcode.config;

public final class TagAimTuning {
    private TagAimTuning() {}

    public static double KP = 0.02;           // Proportional gain converting tag bearing error to twist
    public static double KD = 0.003;          // Derivative gain tempering overshoot in twist response
    public static double CLAMP_ABS = 0.6;     // Max absolute twist from TagAimController (±CLAMP_ABS)
    public static double DEADBAND_DEG = 1.5;  // Heading window treated as on-target (deg)
}
