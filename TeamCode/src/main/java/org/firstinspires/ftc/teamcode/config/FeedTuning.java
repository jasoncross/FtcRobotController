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
 *         rings stick; drop toward 0.7 if jams become frequent.
 *   - FIRE_TIME_MS
 *       • Duration of each feed pulse. Coordinate with SHOT_BETWEEN_MS so the
 *         launcher has time to recover.
 *   - MIN_CYCLE_MS
 *       • Minimum delay between feeds, preventing rapid double-fires even if the
 *         driver mashes the button.
 */
package org.firstinspires.ftc.teamcode.config;

public final class FeedTuning {
    private FeedTuning() {}

    public static double FIRE_POWER = 0.9;
    public static int FIRE_TIME_MS  = 600;
    public static int MIN_CYCLE_MS  = 300;
}
