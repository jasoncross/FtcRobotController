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
 *       • Timeout used when shot detection fails; feed stops when encoder drop
 *         or this timeout is reached.
 *   - MIN_CYCLE_MS
 *       • Legacy cooldown between feeds; still referenced by some Auto routines.
 *   - FIRE_POWER_LAUNCHING
 *       • Reduced feed power after shot detection to prevent shove.
 *   - FIRING_DROP_RPM_THRESHOLD
 *       • RPM drop from target that indicates a ball has entered the launcher.
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
    // CHANGES (2026-01-04): Added RPM drop detection + post-shot feed power controls for shared firing.
    public static double FIRE_POWER = 0.8;          // Motor power applied during each feed pulse
    public static int FIRE_TIME_MS  = 1300;         // Timeout (ms) before assuming shot fired when no RPM drop is detected
    public static int MIN_CYCLE_MS  = 1000;         // Legacy spacing (ms) between successive feed pulses (Auto only)
    public static double FIRE_POWER_LAUNCHING = 0.10; // Reduced feed power after shot detection to prevent shove
    public static double FIRING_DROP_RPM_THRESHOLD = 600; // RPM drop from target to declare a shot captured
    public static int INTAKE_ASSIST_MS = 250; // Intake assist duration (ms) after a feed when intake was off
    public static double IDLE_HOLD_POWER = -0.1; // Idle counter-rotation power (0 = BRAKE only)
}
