/*
 * FILE: TeleOpDriverDefaults.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Gather TeleOp-only driver workflow knobs—startup toggles, manual RPM range,
 *     braking floor, auto-aim grace window, and auto-stop timer—so changing how
 *     TeleOp feels never requires editing TeleOpAllianceBase.
 *   - Mirror the structure of TunableDirectory tables so drivers know exactly
 *     where to tweak common preferences.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Intake power & driver defaults)
 *   - AUTO_SPEED_ENABLED / AUTO_AIM_ENABLED / INTAKE_ENABLED
 *       • Startup states for the respective systems when TeleOp init completes.
 *   - SLOWEST_SPEED
 *       • Minimum drive power while the brake trigger is held (scales twist and
 *         translation equally).
 *   - RPM_BOTTOM / RPM_TOP
 *       • Manual launcher range when AutoSpeed is off. Ensure RPM_TOP ≤
 *         LauncherTuning.RPM_MAX so clamps do not fight each other.
 *   - AUTO_AIM_LOSS_GRACE_MS
 *       • Grace window before AutoAim disables itself after losing a tag.
 *   - TELEMETRY_SMOOTH_A
 *       • Low-pass constant for range/heading telemetry displayed to drivers.
 *   - RPM_TEST_STEP
 *       • Increment applied when D-pad left/right adjust manual RPM in test mode.
 *   - AUTO_STOP_TIMER_ENABLED / AUTO_STOP_TIMER_TIME_SEC
 *       • Optional end-of-match safety timer configuration.
 *   - TELEOP_TELEMETRY_DEBUG_ENABLED
 *       • Dashboard + gamepad-controlled toggle for below-line debug telemetry.
 *   - TELEOP_TELEMETRY_BELOW_HZ
 *       • Rate limit for TeleOp telemetry below the blank separator.
 *   - DEBUG_FIRING_STATS
 *       • Enables debug-only launcher RPM drop/recovery telemetry for fired shots.
 *   - DEBUG_FIRING_STATS_TRIGGER
 *       • RPM below target required to mark a shot as "dropped" for timing.
 *   - DEBUG_FIRING_STATS_VAR_TIME
 *       • Time window (ms) used to compute left/right RPM variance at rest.
 *   - INTAKE_ASSIST_MS
 *       • TeleOp copy of FeedTuning.INTAKE_ASSIST_MS; adjust here when diverging
 *         from the shared value.
 *   - INITIAL_AUTO_DEFAULT_SPEED
 *       • Local override of SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED for
 *         TeleOp-only experiments.
 */
package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class TeleOpDriverDefaults {
    private TeleOpDriverDefaults() {}

    // CHANGES (2025-10-30): Point intake assist mirror at FeedTuning after tunable move.
    // CHANGES (2025-10-31): Default AutoSpeed + Intake to ON per driver request.
    // CHANGES (2025-11-23): Added AutoRPM tweak scale for on-the-fly AutoSpeed adjustments via D-pad left/right.
    // CHANGES (2025-11-25): Added triple-tap intake reverse gesture window for RB and aligned changelog dates.
    // CHANGES (2026-01-03): Added TeleOp telemetry debug toggle + below-line update rate tunables.
    // CHANGES (2026-01-03): Added a debug firing stats toggle for launcher drop/recovery telemetry.
    // CHANGES (2026-01-04): Added firing stats drop trigger and variance window tunables.
    // Startup toggles
    public static boolean AUTO_SPEED_ENABLED = true;  // TeleOp init default for AutoSpeed toggle
    public static boolean AUTO_AIM_ENABLED   = false; // TeleOp init default for AutoAim toggle
    public static boolean INTAKE_ENABLED     = true;  // TeleOp init default for intake run state

    // Drive brake floor
    public static double SLOWEST_SPEED = 0.25;        // Minimum drive power while brake trigger held

    // Manual RPM slider bounds
    public static double RPM_BOTTOM = 0.0;            // Manual RPM floor when AutoSpeed is off
    public static double RPM_TOP    = 6000.0;         // Manual RPM ceiling when AutoSpeed is off
    public static double AUTORPM_TWEAK_SCALE = 0.02;  // Fractional AutoRPM nudge applied per D-pad press while AutoSpeed is on

    // AutoAim grace + telemetry smoothing
    public static int    AUTO_AIM_LOSS_GRACE_MS = 4000; // Grace window (ms) before AutoAim disables after tag loss
    public static double TELEMETRY_SMOOTH_A     = 0.25;  // Smoothing alpha for range/heading telemetry

    // RPM test adjustments (D-pad)
    public static double RPM_TEST_STEP = 50.0;         // RPM delta per D-pad press while RPM test mode is active

    // Intake assist + auto-speed seed (TeleOp overrides of shared values)
    public static int    INTAKE_ASSIST_MS           = FeedTuning.INTAKE_ASSIST_MS;           // TeleOp copy of post-feed intake assist duration (ms)
    public static double INITIAL_AUTO_DEFAULT_SPEED = SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED; // TeleOp seed RPM before first tag lock
    public static int    INTAKE_REVERSE_TAP_WINDOW_MS = 750; // Max window (ms) for detecting three fast RB taps to trigger reverse

    // Auto-stop timer preferences
    public static boolean AUTO_STOP_TIMER_ENABLED = false; // Whether the optional end-of-match timer is active
    public static int     AUTO_STOP_TIMER_TIME_SEC = 119;   // Seconds before AutoStop engages when enabled

    // Telemetry controls
    public static boolean TELEOP_TELEMETRY_DEBUG_ENABLED = false; // TeleOp debug telemetry below the separator (dashboard + gamepad controlled)
    public static double  TELEOP_TELEMETRY_BELOW_HZ      = 10.0;  // Update rate (Hz) for below-separator telemetry blocks
    public static boolean DEBUG_FIRING_STATS             = true;  // Debug-only firing stats (RPM drop/recovery timing) when telemetry debug is enabled
    public static double  DEBUG_FIRING_STATS_TRIGGER     = 100.0; // RPM drop below target required to log a firing drop event
    public static int     DEBUG_FIRING_STATS_VAR_TIME    = 1000;  // Window (ms) for computing launcher RPM variance while idle
}
