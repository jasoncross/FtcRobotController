/*
 * FILE: DebugTelemetryConfig.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Centralize TeleOp debug telemetry defaults and per-system enable flags
 *     so teams can keep deep diagnostics off during matches while still
 *     toggling individual debug blocks during tuning.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Intake power & driver defaults)
 *   - TELEOP_TELEMETRY_DEBUG_ENABLED
 *       • Default state for below-separator debug telemetry (SELECT + dashboard).
 *   - TELEOP_TELEMETRY_BELOW_HZ
 *       • Update rate for below-separator telemetry blocks.
 *   - ENABLE_FIRING_STATE_DEBUG
 *       • Forces firing-state timing + readiness telemetry below the separator.
 *   - DEBUG_FIRING_STATS
 *       • Enables launcher RPM drop/recovery telemetry after shots fire.
 *   - DEBUG_FIRING_STATS_TRIGGER
 *       • RPM below target required to mark a shot drop event.
 *   - DEBUG_FIRING_STATS_VAR_TIME
 *       • Time window (ms) used to compute left/right RPM variance at rest.
 *   - DEBUG_FIRING_TELEMETRY
 *       • Enables firing state/mode lines inside the debug block.
 *   - DEBUG_AIM_TELEMETRY
 *       • Enables AutoAim/goal-tag diagnostics inside the debug block.
 *   - DEBUG_AUTORPM_TELEMETRY
 *       • Enables AutoRPM calibration and smoothing debug lines.
 *   - DEBUG_VISION_TELEMETRY
 *       • Enables vision status/health/lighting diagnostics in debug telemetry.
 *   - DEBUG_FEEDSTOP_TELEMETRY
 *       • Enables FeedStop clamp/home diagnostics in debug telemetry.
 *   - DEBUG_LIMELIGHT_AUTOSELECT
 *       • Enables Limelight auto-select status lines in debug telemetry.
 *   - DEBUG_ODOMETRY_TELEMETRY
 *       • Enables odometry + vision fusion debug lines in debug telemetry.
 */
package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class DebugTelemetryConfig {
    private DebugTelemetryConfig() {}

    // CHANGES (2026-01-10): Moved TeleOp debug telemetry defaults and per-system debug flags into a dedicated config file.
    public static boolean TELEOP_TELEMETRY_DEBUG_ENABLED = false; // Default TeleOp debug telemetry state (SELECT + dashboard)
    public static double  TELEOP_TELEMETRY_BELOW_HZ      = 10.0;  // Update rate (Hz) for below-separator telemetry blocks

    public static boolean ENABLE_FIRING_STATE_DEBUG   = true;  // Always show firing state timing/readiness block below separator
    public static boolean DEBUG_FIRING_STATS          = true;  // Enable launcher RPM drop/recovery + variance debug telemetry
    public static double  DEBUG_FIRING_STATS_TRIGGER  = 100.0; // RPM drop below target required to log a firing drop event
    public static int     DEBUG_FIRING_STATS_VAR_TIME = 1000;  // Window (ms) for computing launcher RPM variance while idle

    public static boolean DEBUG_FIRING_TELEMETRY       = true; // Show firing state/mode diagnostics in debug telemetry
    public static boolean DEBUG_AIM_TELEMETRY          = true; // Show AutoAim + tag-visibility diagnostics in debug telemetry
    public static boolean DEBUG_AUTORPM_TELEMETRY      = true; // Show AutoRPM calibration + smoothing diagnostics in debug telemetry
    public static boolean DEBUG_VISION_TELEMETRY       = true; // Show vision status, health, and lighting diagnostics in debug telemetry
    public static boolean DEBUG_FEEDSTOP_TELEMETRY     = true; // Show FeedStop clamp/home diagnostics in debug telemetry
    public static boolean DEBUG_LIMELIGHT_AUTOSELECT   = true; // Show Limelight auto-select profile status in debug telemetry
    public static boolean DEBUG_ODOMETRY_TELEMETRY     = true; // Show odometry + vision fusion debug lines in debug telemetry
}
