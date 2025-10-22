// ====================================================================================================
//  FILE:           RumbleNotifier.java
//  LOCATION:       org.firstinspires.ftc.teamcode.util
//  PURPOSE:        Provides controller haptic feedback (rumble) when the robot is aimed within
//                  a specified angular threshold of the detected AprilTag target.
//
//  NOTES:
//      - Uses FTC SDK's Gamepad rumble API to give tactile feedback to the driver.
//      - Can be configured for strength, pulse length, and cooldown between pulses.
//      - Ideal for use in TeleOp when aligning with field targets using AprilTags.
//      - Designed to be updated once per control loop.
//
//  USAGE EXAMPLE:
//      RumbleNotifier rumble = new RumbleNotifier(gamepad1);
//      rumble.setThresholdDeg(1.0);      // Buzz when within ±1 degree
//      rumble.setStrength(0.6);          // 60% rumble strength
//      rumble.update(yawError, visible); // Call each loop with tag data
//
//  DEPENDENCIES:
//      - Requires valid Gamepad instance (e.g. gamepad1 or gamepad2)
//      - Vision or targeting system must provide yaw error (in degrees) and visibility flag.
//
//  CREATED:        October 2025
//  AUTHORS:        Indianola Robotics – FTC TeamCode (DECODE Season)
//
//  METHODS:
//      • setThresholdDeg(double)     → sets angular tolerance for rumble activation.
//      • setStrength(double)         → sets rumble intensity (0.0–1.0).
//      • setPulseMs(long)            → sets pulse duration per rumble event.
//      • setCooldownMs(long)         → sets minimum time between pulses.
//      • update(double, boolean)     → checks if yaw error is within threshold and triggers rumble.
// ====================================================================================================

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import android.os.SystemClock;

public class RumbleNotifier {

    // --- Instance Variables ---
    private final Gamepad gamepad;         // Gamepad reference for rumble control
    private double thresholdDeg = 1.0;     // Default angular window in degrees (±)
    private long cooldownMs = 250;         // Delay between rumble pulses (ms)
    private long pulseMs = 180;            // Duration of a single rumble pulse (ms)
    private double strength = 0.6;         // Intensity (0.0–1.0)
    private long lastBuzzMs = 0;           // Timestamp of last rumble event
    private boolean insideWindowLastLoop = false; // Tracks whether last loop was within threshold

    // ------------------------------------------------------------------------------------------------
    //  CONSTRUCTOR:  RumbleNotifier(Gamepad gamepad)
    //  PURPOSE:      Creates a new notifier tied to the specified controller.
    // ------------------------------------------------------------------------------------------------
    public RumbleNotifier(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    // ------------------------------------------------------------------------------------------------
    //  CONFIGURATION METHODS
    // ------------------------------------------------------------------------------------------------
    public void setThresholdDeg(double deg) { this.thresholdDeg = Math.max(0, deg); }
    public double getThresholdDeg() { return thresholdDeg; }

    public void setStrength(double s) { this.strength = Math.max(0, Math.min(1, s)); }
    public void setPulseMs(long ms) { this.pulseMs = Math.max(20, ms); }
    public void setCooldownMs(long ms) { this.cooldownMs = Math.max(0, ms); }

    // ------------------------------------------------------------------------------------------------
    //  METHOD:       update(double yawErrorDeg, boolean tagVisible)
    //  PURPOSE:      Evaluates the current yaw error to determine if the robot is aligned
    //                with the AprilTag target. When within threshold, triggers a controller rumble.
    //
    //  PARAMETERS:
    //      yawErrorDeg  → Signed angular error (in degrees) from tag centerline.
    //                     Use NaN if no valid target is detected.
    //      tagVisible   → True if the target is currently detected by vision.
    //
    //  OPERATION:
    //      1. If tag is visible and yaw error is within threshold, rumble is triggered.
    //      2. Rumble will not repeat until cooldown period has passed.
    //      3. The gamepad’s left and right motors both activate at the same intensity.
    // ------------------------------------------------------------------------------------------------
    public void update(double yawErrorDeg, boolean tagVisible) {
        boolean inside = tagVisible && !Double.isNaN(yawErrorDeg) && Math.abs(yawErrorDeg) <= thresholdDeg;
        long now = SystemClock.uptimeMillis();

        if (inside) {
            boolean cooled = now - lastBuzzMs >= cooldownMs;
            if (!insideWindowLastLoop || cooled) {
                // Trigger rumble (both motors)
                // If your SDK version supports runRumbleEffect(), that can be used for patterns.
                gamepad.rumble(strength, strength, pulseMs);
                lastBuzzMs = now;
            }
        }

        insideWindowLastLoop = inside;
    }
}
