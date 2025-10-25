/*
 * FILE: RumbleNotifier.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/
 *
 * PURPOSE
 *   - Provide SDK-compatible rumble helpers for toggle feedback and aim-window
 *     cues without relying on newer Gamepad APIs.
 *   - Pulse the controller with tunable strength, duration, and cooldown based
 *     on heading error.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Driver feedback)
 *   - thresholdDeg
 *       • Heading error window that triggers pulses. Tighten toward 1.5° for
 *         earlier feedback, relax toward 3° if distracting.
 *   - min/maxStrength, min/maxPulseMs, min/maxCooldownMs
 *       • Bounds for rumble intensity, pulse length, and cooldown. TeleOpAllianceBase
 *         overrides these on init (e.g., 0.05–0.80 strength, 80–160 ms pulses).
 *
 * METHODS
 *   - setActive()/isActive()
 *       • Master enable toggle.
 *   - setThresholdDeg(), setMinMax(...)
 *       • Configure behavior to match TeleOp preferences.
 *   - update(errorDeg)
 *       • Call every loop; emits pulses when within the threshold.
 *
 * NOTES
 *   - Uses the simple gp.rumble(left,right,durationMs) call so it works across
 *     SDK versions and both gamepads.
 *   - All setters clamp values defensively to keep rumble sane even if a config
 *     mistake slips through.
 */
package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class RumbleNotifier {

    // --- Config (tunable via setters) ---
    private boolean active = true;            // master enable
    private double  thresholdDeg = 2.5;       // begin rumble when |error| <= threshold

    private double  minStrength = 0.10;       // 0..1
    private double  maxStrength = 0.65;       // 0..1

    private int     minPulseMs = 120;         //  ms
    private int     maxPulseMs = 200;         //  ms

    private int     minCooldownMs = 120;      //  ms
    private int     maxCooldownMs = 350;      //  ms

    // --- Runtime ---
    private final Gamepad gp;
    private long lastPulseAtMs = 0L;

    public RumbleNotifier(Gamepad gp) {
        this.gp = gp;
    }

    // =========================
    // PUBLIC CONFIG METHODS
    // =========================
    public void setActive(boolean enable) { this.active = enable; }
    public boolean isActive() { return this.active; }

    public void setThresholdDeg(double deg) {
        if (Double.isNaN(deg) || deg <= 0) return;
        this.thresholdDeg = deg;
    }

    /**
     * Configure scaling ranges for strength, pulse length, and cooldown.
     * All values are clamped to sane bounds.
     */
    public void setMinMax(double minStr, double maxStr,
                          int minPulseMs, int maxPulseMs,
                          int minCooldownMs, int maxCooldownMs) {
        // Strength 0..1
        this.minStrength = clamp(minStr, 0.0, 1.0);
        this.maxStrength = clamp(maxStr, 0.0, 1.0);
        if (this.maxStrength < this.minStrength) {
            double t = this.minStrength; this.minStrength = this.maxStrength; this.maxStrength = t;
        }

        // Pulse duration
        this.minPulseMs = clamp(minPulseMs, 20, 2000);
        this.maxPulseMs = clamp(maxPulseMs, 20, 2000);
        if (this.maxPulseMs < this.minPulseMs) {
            int t = this.minPulseMs; this.minPulseMs = this.maxPulseMs; this.maxPulseMs = t;
        }

        // Cooldown
        this.minCooldownMs = clamp(minCooldownMs, 20, 5000);
        this.maxCooldownMs = clamp(maxCooldownMs, 20, 5000);
        if (this.maxCooldownMs < this.minCooldownMs) {
            int t = this.minCooldownMs; this.minCooldownMs = this.maxCooldownMs; this.maxCooldownMs = t;
        }
    }

    // =========================
    // PUBLIC UPDATE
    // =========================
    /**
     * Call periodically with current heading error (deg).
     * Emits a short rumble pulse when |error| <= thresholdDeg and cooldown elapsed.
     * Safe to call at any rate; internal cooldown avoids over-rumble.
     */
    public void update(double headingErrorDeg) {
        if (!active || gp == null) return;

        double absErr = Math.abs(headingErrorDeg);
        if (Double.isNaN(absErr)) return;

        if (absErr > thresholdDeg) {
            // Outside window → do nothing (no hum)
            return;
        }

        long now = System.currentTimeMillis();
        int cooldown = lerpI(absErr / thresholdDeg, minCooldownMs, maxCooldownMs);
        if (now - lastPulseAtMs < cooldown) return;

        // Map smaller error → bigger strength & longer pulse.
        double strength = lerpD(absErr / thresholdDeg, minStrength, maxStrength);
        int durationMs  = lerpI(absErr / thresholdDeg, minPulseMs, maxPulseMs);

        // FTC older SDK-compatible rumble call
        gp.rumble((float)strength, (float)strength, durationMs);

        lastPulseAtMs = now;
    }

    // =========================
    // HELPERS
    // =========================
    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // t expected 0..1 (but we clamp defensively)
    private static double lerpD(double t, double a, double b) {
        t = clamp(t, 0.0, 1.0);
        return a + (b - a) * (1.0 - t); // note: invert so smaller error → larger output
    }

    private static int lerpI(double t, int a, int b) {
        return (int)Math.round(lerpD(t, a, b));
    }
}
