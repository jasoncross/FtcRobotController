/*
 * FILE: FeedTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Own the shared feed motor parameters used by both TeleOp and Autonomous so
 *     cadence tweaks happen in one place instead of inside Feed.java.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Shot cadence, feed, and eject)
 *   - FIRE_POWER
 *       • Motor power applied while the feed pulse runs. Increase toward 1.0 when
 *         artifacts stick; drop toward 0.7 if jams become frequent.
 *   - FIRE_TIME_MS
 *       • Duration of each feed pulse. Ensure autonomous `betweenShotsMs`
 *         delays leave enough time for RPM recovery.
 *   - MIN_CYCLE_MS
 *       • Minimum delay between feeds, preventing rapid double-fires even if the
 *         driver mashes the button.
 *   - INTAKE_ASSIST_MS (MOVED 2025-10-30)
 *       • Duration TeleOp/Auto run the intake after a feed when it was previously
 *         off. Centralized here alongside other feed cadence values.
 *   - IDLE_HOLD_POWER (ADDED 2025-10-31)
 *       • Counter-rotation power applied while idle to keep the feed staged.
 *         Set to 0 to fall back to BRAKE-only holding.
 */
package org.firstinspires.ftc.teamcode.config;

public final class FeedTuning {
    private FeedTuning() {}

    // CHANGES (2025-10-30): Added INTAKE_ASSIST_MS after moving ownership from SharedRobotTuning.
    // CHANGES (2025-10-31): Added IDLE_HOLD_POWER to keep the feed counter-rotating while idle.
    // CHANGES (2025-10-31): Increased IDLE_HOLD_POWER magnitude so staged artifacts remain latched.
    // CHANGES (2025-11-02): Updated documentation to reflect per-sequence shot cadence control.
    public static double FIRE_POWER = 0.8;          // Motor power applied during each feed pulse
    public static int FIRE_TIME_MS  = 800;          // Duration (ms) to run the feed motor per shot
    public static int MIN_CYCLE_MS  = 1000;         // Minimum spacing (ms) between successive feed pulses
    public static int INTAKE_ASSIST_MS = 250; // Intake assist duration (ms) after a feed when intake was off
    public static double IDLE_HOLD_POWER = -0.1; // Idle counter-rotation power (0 = BRAKE only)
}
