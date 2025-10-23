// ====================================================================================================
//  FILE:           RumbleNotifier.java
//  LOCATION:       org.firstinspires.ftc.teamcode.util
//  PURPOSE:        Haptic feedback for AprilTag aiming that ADAPTS with accuracy.
//                  • Within ±thresholdDeg: pulses the gamepad.
//                  • Intensity grows from minStrength (at edge) to maxStrength (at 0°).
//                  • Pulse duration increases toward center.
//                  • Cooldown shrinks toward center → faster pulse cadence.
//
//  NOTES:
//      - Call update(yawErrorDeg, tagVisible) once per loop.
//      - Works with FTC SDK Gamepad rumble(left,right,int durationMs).
//      - If the controller lacks rumble motors, calls are no-ops.
//      - All ranges are safely clamped.
//
//  CONFIG SURFACE:
//      • thresholdDeg (± window in degrees)
//      • setStrengthRange(min, max)   : 0..1
//      • setPulseRange(minMs, maxMs)  : 20..2000 ms typical
//      • setCooldownRange(minMs, maxMs): 0..2000 ms typical
//
//  METHODS:
//      • setThresholdDeg(double)
//      • setStrengthRange(double min, double max)
//      • setPulseRange(int minMs, int maxMs)
//      • setCooldownRange(int minMs, int maxMs)
//      • update(double yawErrorDeg, boolean tagVisible)
// ====================================================================================================

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import android.os.SystemClock;

public class RumbleNotifier {

    // --- Configuration ---
    private final Gamepad gamepad;

    private double thresholdDeg      = 1.0;   // ± degrees window
    private double minStrength       = 0.25;  // strength at window edge
    private double maxStrength       = 0.85;  // strength at 0° error

    private int    minPulseMs        = 120;   // pulse length at edge
    private int    maxPulseMs        = 200;   // pulse length at center

    private int    minCooldownMs     = 120;   // shortest gap (near center)
    private int    maxCooldownMs     = 350;   // longest gap (at edge)

    // --- State ---
    private long   lastBuzzMs        = 0;
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

    /** 0..1, clamped and ordered. */
    public void setStrengthRange(double min, double max) {
        double lo = clamp01(min);
        double hi = clamp01(max);
        if (hi < lo) { double t = lo; lo = hi; hi = t; }
        this.minStrength = lo;
        this.maxStrength = hi;
    }

    /** Pulse duration range in ms (min ≤ max). */
    public void setPulseRange(int minMs, int maxMs) {
        int lo = Math.max(20, minMs);
        int hi = Math.max(lo, maxMs);
        this.minPulseMs = lo;
        this.maxPulseMs = hi;
    }

    /** Cooldown (gap) range in ms (min ≤ max). */
    public void setCooldownRange(int minMs, int maxMs) {
        int lo = Math.max(0, minMs);
        int hi = Math.max(lo, maxMs);
        this.minCooldownMs = lo;
        this.maxCooldownMs = hi;
    }

    // ------------------------------------------------------------------------------------------------
    //  UPDATE (call once per loop)
    //  Maps |error| within [0..threshold] to:
    //    frac = 1 - |err|/threshold  (edge→0.0 … center→1.0)
    //    strength = lerp(minStrength, maxStrength, frac)
    //    pulseMs  = lerp(minPulseMs,  maxPulseMs,  frac)
    //    cooldown = lerp(maxCooldownMs, minCooldownMs, frac)  // invert: shorter near center
    //  Triggers a pulse when entering window or after cooldown expires.
// ------------------------------------------------------------------------------------------------
    public void update(double yawErrorDeg, boolean tagVisible) {
        boolean valid = tagVisible && !Double.isNaN(yawErrorDeg) && thresholdDeg > 0;
        boolean inside = valid && Math.abs(yawErrorDeg) <= thresholdDeg;

        long now = SystemClock.uptimeMillis();

        if (inside) {
            double absErr = Math.abs(yawErrorDeg);
            double frac = 1.0 - (absErr / thresholdDeg);      // 0..1
            frac = clamp01(frac);

            double strength = lerp(minStrength,  maxStrength,  frac);
            int    pulseMs  = (int)Math.round(lerp(minPulseMs, maxPulseMs, frac));
            int    cooldown = (int)Math.round(lerp(maxCooldownMs, minCooldownMs, frac)); // invert cadence

            boolean cooled = (now - lastBuzzMs) >= cooldown;

            if (!insideWindowLastLoop || cooled) {
                gamepad.rumble(strength, strength, pulseMs);
                lastBuzzMs = now;
            }
        }

        insideWindowLastLoop = inside;
    }

    // ------------------------------------------------------------------------------------------------
    //  HELPERS
    // ------------------------------------------------------------------------------------------------
    private static double clamp01(double v) { return Math.max(0.0, Math.min(1.0, v)); }
    private static double lerp(double a, double b, double t) { return a + (b - a) * t; }
}
