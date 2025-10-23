// ============================================================================
// FILE:           RumbleNotifier.java
// LOCATION:       TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/
//
// PURPOSE:
//   Small helper for controller rumble behaviors:
//   - Toggle pulses (enable/disable haptics cues)
//   - Optional “aim window” rumble that scales with heading error
//
// NOTES:
//   This class is intentionally lightweight so it compiles on a wide range
//   of FTC SDK versions. It does NOT depend on Gamepad.RumbleEffect in looped
//   logic; only simple rumble(duration) calls are used here.
//   TeleOp handles its own enable/disable pulses.
//
// METHODS:
//   RumbleNotifier(Gamepad gp)
//   setActive(boolean enable)
//   setThresholdDeg(double deg)
//   setMinMax(double minStrength, double maxStrength,
//             int minPulseMs, int maxPulseMs,
//             int minCooldownMs, int maxCooldownMs)
//   update(double headingErrorDeg)   // call periodically if you want aim-rumble
// ============================================================================
package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class RumbleNotifier {

    private final Gamepad gamepad;

    // Master enable for aim-rumble behavior
    private boolean active = false;

    // Angle at/inside which rumble begins to appear
    private double thresholdDeg = 2.5;

    // Strength 0..1
    private double minStrength = 0.10;
    private double maxStrength = 0.65;

    // Pulse/cooldown ranges (milliseconds)
    private int minPulseMs     = 120;
    private int maxPulseMs     = 200;
    private int minCooldownMs  = 120;
    private int maxCooldownMs  = 350;

    // Internal timing state
    private long nextAllowedMs = 0L;

    public RumbleNotifier(Gamepad gp) {
        this.gamepad = gp;
    }

    /** Master on/off for aim-rumble behavior. */
    public void setActive(boolean enable) {
        this.active = enable;
    }

    /** Degrees from 0 where haptics begin to kick in. */
    public void setThresholdDeg(double deg) {
        this.thresholdDeg = Math.max(0, deg);
    }

    /**
     * Configure strength and timing ranges for adaptive aim-rumble.
     *
     * @param minStrength     0..1 minimum rumble intensity
     * @param maxStrength     0..1 maximum rumble intensity
     * @param minPulseMs      minimum pulse duration in ms
     * @param maxPulseMs      maximum pulse duration in ms
     * @param minCooldownMs   minimum cooldown between pulses in ms
     * @param maxCooldownMs   maximum cooldown between pulses in ms
     */
    public void setMinMax(double minStrength, double maxStrength,
                          int minPulseMs, int maxPulseMs,
                          int minCooldownMs, int maxCooldownMs) {
        this.minStrength    = clamp01(minStrength);
        this.maxStrength    = clamp01(maxStrength);
        this.minPulseMs     = Math.max(1, minPulseMs);
        this.maxPulseMs     = Math.max(this.minPulseMs, maxPulseMs);
        this.minCooldownMs  = Math.max(1, minCooldownMs);
        this.maxCooldownMs  = Math.max(this.minCooldownMs, maxCooldownMs);
    }

    /** Call periodically with signed heading error in degrees (target is 0°). */
    public void update(double headingErrorDeg) {
        if (!active) return;

        double absErr = Math.abs(headingErrorDeg);
        if (absErr > thresholdDeg) return; // outside window → no rumble

        long now = System.currentTimeMillis();
        if (now < nextAllowedMs) return;

        // Map error → intensity (closer to 0° = stronger rumble) and pulse/cooldown
        double t = 1.0 - clamp01(absErr / Math.max(1e-6, thresholdDeg));
        double strength = lerp(minStrength, maxStrength, t);
        int pulseMs     = (int)Math.round(lerp(minPulseMs,  maxPulseMs,  t));
        int cooldownMs  = (int)Math.round(lerp(minCooldownMs, maxCooldownMs, t));

        // Simple one-shot pulse (SDK compatible)
        gamepad.rumble((float)strength, (float)strength, pulseMs);
        nextAllowedMs = now + pulseMs + cooldownMs;
    }

    // ------------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------------
    private static double clamp01(double v) { return Math.max(0.0, Math.min(1.0, v)); }
    private static double lerp(double a, double b, double t) { return a + (b - a) * t; }
}
