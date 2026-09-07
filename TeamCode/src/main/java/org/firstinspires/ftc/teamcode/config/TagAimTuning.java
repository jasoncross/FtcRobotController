/** Retained frame-delta PD gains, power clamp and target bearing deadband; retune on the new robot. */
package org.firstinspires.ftc.teamcode.config;

public final class TagAimTuning {
    private TagAimTuning() {}

    public static double KP = 0.02;           // Proportional gain converting tag bearing error to twist
    public static double KD = 0.003;          // Derivative gain tempering overshoot in twist response
    public static double CLAMP_ABS = 0.6;     // Max absolute twist from TagAimController (±CLAMP_ABS)
    public static double DEADBAND_DEG = 1.5;  // Heading window treated as on-target (deg)
}
