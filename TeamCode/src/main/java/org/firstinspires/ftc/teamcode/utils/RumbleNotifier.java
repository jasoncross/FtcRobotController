// ====================================================================================================
//  FILE:           RumbleNotifier.java
//  LOCATION:       org.firstinspires.ftc.teamcode.util
//  PURPOSE:        Haptic feedback that scales with aiming accuracy. When the absolute yaw error
//                  to the target is within a ±threshold, it rumbles the controller. Intensity grows
//                  from minStrength (at the window edge) up to maxStrength (at 0° error).
//
//  NOTES:
//      - Uses FTC SDK Gamepad rumble(left,right,int durationMs).
//      - Cooldown prevents constant buzzing when hovering at threshold.
//      - Call update(...) once per loop with current yaw error and visibility.
//      - If your controller lacks rumble hardware (e.g., some Logitech pads), nothing will happen.
//
//  CONFIG:
//      • thresholdDeg     : ±degrees for “on target” window.
//      • minStrength/maxStrength : intensity range [0..1] mapped from edge→center.
//      • pulseMs          : pulse duration (ms).
//      • cooldownMs       : time between pulses (ms).
//
//  METHODS:
//      • setThresholdDeg(double)
//      • setStrengthRange(double min, double max)
//      • setPulseMs(int), setCooldownMs(int)
//      • update(double yawErrorDeg, boolean tagVisible)
// ====================================================================================================

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import android.os.SystemClock;

public class RumbleNotifier {

    // --- Instance Variables ---
    private final Gamepad gamepad;

    private double thresholdDeg = 1.0;     // ± degrees window
    private double minStrength  = 0.25;    // rumble intensity at the window edge
    private double maxStrength  = 0.80;    // rumble intensity at 0° error

    private int    pulseMs      = 180;     // duration per buzz (ms)
    private int    cooldownMs   = 250;     // min gap between buzzes (ms)

    private long   lastBuzzMs   = 0;       // last buzz time
    private boolean insideWindowLastLoop = false;

    // ------------------------------------------------------------------------------------------------
    //  CONSTRUCTOR
    // ------------------------------------------------------------------------------------------------
    public RumbleNotifier(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    // ------------------------------------------------------------------------------------------------
    //  CONFIGURATION
    // ------------------------------------------------------------------------------------------------
    public void setThresholdDeg(double deg) {
        this.thresholdDeg = Math.max(0, deg);
    }
    public double getThresholdDeg() { return thresholdDeg; }

    /** Sets the intensity range; values are clamped to [0..1] and sorted (min ≤ max). */
    public void setStrengthRange(double min, double max) {
        double lo = clamp01(min);
        double hi = clamp01(max);
        if (hi < lo) { double t = lo; lo = hi; hi = t; }
        this.minStrength = lo;
        this.maxStrength = hi;
    }

    public void setPulseMs(int ms)    { this.pulseMs = Math.max(20, ms); }
    public void setCooldownMs(int ms) { this.cooldownMs = Math.max(0, ms); }

    // ------------------------------------------------------------------------------------------------
    //  UPDATE: Call once per loop with current yaw error (deg) and target visibility.
    //  • Computes scaled intensity = min + (max-min)*(1 - |err|/threshold), clamped 0..1.
    //  • Triggers a short rumble pulse when entering the window or after cooldown.
// ------------------------------------------------------------------------------------------------
    public void update(double yawErrorDeg, boolean tagVisible) {
        boolean valid = tagVisible && !Double.isNaN(yawErrorDeg) && thresholdDeg > 0;
        boolean inside = valid && Math.abs(yawErrorDeg) <= thresholdDeg;

        long now = SystemClock.uptimeMillis();
        if (inside) {
            // Map |error| within [0..threshold] to intensity within [minStrength..maxStrength]
            double absErr = Math.abs(yawErrorDeg);
            double frac = 1.0 - (absErr / thresholdDeg);     // edge→0.0 … center→1.0
            frac = clamp01(frac);

            double strength = minStrength + (maxStrength - minStrength) * frac;
            strength = clamp01(strength);

            boolean cooled = (now - lastBuzzMs) >= cooldownMs;
            if (!insideWindowLastLoop || cooled) {
                gamepad.rumble(strength, strength, (int) pulseMs);  // duration expects int
                lastBuzzMs = now;
            }
        }

        insideWindowLastLoop = inside;
    }

    // ------------------------------------------------------------------------------------------------
    //  HELPERS
    // ------------------------------------------------------------------------------------------------
    private static double clamp01(double v) { return Math.max(0.0, Math.min(1.0, v)); }
}
