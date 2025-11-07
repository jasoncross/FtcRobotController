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
 *   - SCALE_MIN / SCALE_MAX
 *       • Servo PWM bounds passed to ServoImplEx.scaleRange(). Narrowing this
 *         range reduces travel arc and speeds transitions.
 *   - BLOCK_POS / RELEASE_POS
 *       • Normalized positions (0–1) inside the scaled range for closed/open
 *         states. Adjust to align with the physical stop locations.
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
    public static double SCALE_MIN = 0.8; // Minimum PWM range (tune on-robot)
    public static double SCALE_MAX = 1.5; // Maximum PWM range (tune on-robot)
    public static double BLOCK_POS = 0.00; // Servo position (scaled) for blocking artifacts
    public static double RELEASE_POS = 1.20; // Servo position (scaled) for releasing artifacts
    public static long RELEASE_HOLD_MS = 1000; // Duration to remain at RELEASE (ms)
    public static long FIRE_LEAD_MS = 500;     // Delay before feed motor starts (ms)
}
