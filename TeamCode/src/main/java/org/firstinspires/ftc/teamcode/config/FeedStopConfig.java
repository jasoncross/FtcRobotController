/*
 * FILE: FeedStopConfig.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Centralize FeedStop servo tuning so TeleOp and Autonomous share the same
 *     travel limits, block/release setpoints, and release timing.
 *   - Allow on-robot adjustments to narrow the PWM range for faster movement
 *     while keeping code free of hard-coded numbers.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Shot cadence, feed, and eject)
 *   - USE_AUTO_SCALE
 *       • When true, Feed computes a ServoImplEx.scaleRange() window that just
 *         fits the requested hold/release angles plus a safety margin. When
 *         false (default) the servo runs its full 300° span with no scaling.
 *   - DIRECTION_SIGN
 *       • Keeps the logical positive direction aligned with physical RELEASE
 *         motion so teams can flip behavior without rewiring.
 *   - SAFE_PRESET_OPEN_DEG / MAX_HOME_TRAVEL_DEG / HOME_BACKOFF_DEG / HOME_DWELL_MS
 *       • The guarded homing approach: how far to open (CW) before reversing
 *         toward the stop, how far the safe sweep may travel before aborting,
 *         how far to back away after seating, and how long to dwell on the
 *         hard stop to establish a repeatable zero.
 *   - HOLD_ANGLE_DEG / RELEASE_ANGLE_DEG
 *       • Target angles (relative to zero) used while blocking the feed path
 *         and fully releasing for a shot.
 *   - SOFT_CCW_LIMIT_DEG / SOFT_CW_LIMIT_DEG
 *       • Software clamps that prevent commands below 0° or above the safe
 *         release sweep; every request is clamped into this band.
 *   - SAFETY_MARGIN_DEG
 *       • Extra clearance applied when auto-expanding the servo scale window so
 *         HOLD/RELEASE angles never demand the full PWM span.
 *   - RELEASE_HOLD_MS
 *       • How long the servo stays at RELEASE after a fire request before
 *         snapping back to BLOCK.
 *   - FIRE_LEAD_MS
 *       • Lead time between requesting RELEASE and starting the feed motor so
 *         the gate clears before the feed wheel pushes.
 */
package org.firstinspires.ftc.teamcode.config;

public final class FeedStopConfig {
    private FeedStopConfig() {}

    // CHANGES (2025-11-06): Initial FeedStop servo tuning defaults for goBILDA 25-3 speed servo.
    // CHANGES (2025-11-07): Added homing + degrees-based control tunables (overshoot, backoff, release angle, direction sign).
    // CHANGES (2025-11-08): Split HOLD vs. RELEASE degree targets, added safety margin auto-scaling, and removed the fixed
    //                       degrees-per-unit constant in favor of runtime span math.
    // CHANGES (2025-11-09): Default to full-span servo travel, added USE_AUTO_SCALE toggle, and retired SCALE_MIN/MAX tunables.
    // CHANGES (2025-11-07): Added guarded homing presets (safe open + travel cap) and soft-limit clamps for degrees-based control.
    public static boolean USE_AUTO_SCALE = false;    // Optional servo auto-scaling toggle (default full span)
    public static double DIRECTION_SIGN = +1.0;      // +1 when release is toward higher PWM, -1 when toward lower PWM
    public static double SAFE_PRESET_OPEN_DEG = 40.0; // Target angle used during the homing safe-open approach (deg)
    public static double HOME_OVERSHOOT_DEG = 12.0;  // Legacy alias for SAFE_PRESET_OPEN_DEG (kept for backward compatibility)
    public static double MAX_HOME_TRAVEL_DEG = 120.0; // Maximum degrees allowed during homing safe-open sweep (deg)
    public static double HOME_BACKOFF_DEG = 2.0;     // Distance (deg) to back away from the hard stop after seating
    public static long HOME_DWELL_MS = 120;          // Time (ms) to dwell against the BLOCK stop before backoff
    public static double SOFT_CCW_LIMIT_DEG = 0.0;    // Lowest allowed command (deg relative to home)
    public static double SOFT_CW_LIMIT_DEG = 170.0;   // Highest allowed command (deg relative to home)
    public static double HOLD_ANGLE_DEG = 10.0;     // Hold (blocking) angle relative to the homed zero position (deg)
    public static double RELEASE_ANGLE_DEG = 80.0; // Release angle relative to the homed zero position (deg)
    public static double SAFETY_MARGIN_DEG = 5.0;   // Extra clearance added when sizing the PWM span (deg)
    public static long RELEASE_HOLD_MS = 500;       // Duration to remain at RELEASE (ms)
    public static long FIRE_LEAD_MS = 500;           // Delay before feed motor starts (ms)
}
