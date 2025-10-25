/*
 * FILE: SharedRobotTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Centralize the cross-mode tunables that TeleOp, Auto, and AutoAim helpers
 *     share so cadence, aim limits, and readiness thresholds stay synchronized.
 *   - Replace the scattered constants that previously lived inside
 *     TeleOpAllianceBase and BaseAuto.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md for ranges & examples)
 *   - SHOT_BETWEEN_MS
 *       • Minimum delay between autonomous shots enforced by BaseAuto.fireN().
 *       • Coordinate with Feed.minCycleMs to prevent overlapping feed cycles.
 *   - RPM_TOLERANCE
 *       • Shared ±RPM window considered "at speed" by BaseAuto, AutoAimSpeed,
 *         and TeleOpAllianceBase when no override is provided.
 *       • Keep aligned with Launcher.atSpeedToleranceRPM if you tighten or loosen
 *         precision expectations.
 *   - LOCK_TOLERANCE_DEG
 *       • Bearing tolerance used when declaring an AprilTag lock.
 *       • Ensure Drivebase.TURN_TOLERANCE_DEG and TagAimController gains support
 *         this value to avoid oscillations.
 *   - TURN_TWIST_CAP
 *       • Twist clamp applied inside BaseAuto turning helpers and copied into
 *         AutoAimSpeed.maxTwist unless AutoAim overrides it locally.
 *   - DRIVE_MAX_POWER
 *       • Maximum drive power used by BaseAuto motion helpers.
 *       • TeleOp drive scaling is separate; adjust there for driver feel.
 *   - INTAKE_ASSIST_MS
 *       • Duration TeleOp and Auto run the intake after a feed when it was off.
 *       • TeleOpAllianceBase can override its copy; update both when diverging.
 *   - INITIAL_AUTO_DEFAULT_SPEED
 *       • Seed RPM before the first AprilTag lock when AutoSpeed starts.
 *       • TeleOpAllianceBase copies this; align values so warm-up behavior matches.
 *   - LOGO_DIRECTION / USB_DIRECTION
 *       • Physical mounting orientation of the REV Control Hub IMU.
 *       • Update both when the hub is remounted so +yaw remains CCW on the field.
 *
 * NOTES
 *   - This file intentionally contains constants only. Update them whenever
 *     cadence or aim behavior changes to keep every OpMode synchronized.
 */
package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public final class SharedRobotTuning {
    private SharedRobotTuning() {}

    // --- Shot timing ---
    public static long   SHOT_BETWEEN_MS            = 3000;   // BaseAuto.fireN() delay (ms); coordinate with Feed.minCycleMs

    // --- Launcher speed gate ---
    public static double RPM_TOLERANCE              = 50.0;   // Shared ±RPM window; Launcher.atSpeedToleranceRPM should match

    // --- Aim / drive caps used by Auto helpers (safe defaults) ---
    public static double LOCK_TOLERANCE_DEG         = 1.0;    // Bearing tolerance; keep aligned with Drivebase.TURN_TOLERANCE_DEG
    public static double TURN_TWIST_CAP             = 0.35;   // Twist clamp shared by BaseAuto + AutoAimSpeed unless overridden
    public static double DRIVE_MAX_POWER            = 0.50;   // Max auto drive power; adjust here for global movement speed

    // --- Assist behaviors shared across modes ---
    public static int    INTAKE_ASSIST_MS           = 250;    // Intake assist duration (ms); TeleOpAllianceBase copies this value
    public static double INITIAL_AUTO_DEFAULT_SPEED = 2500.0; // Seed RPM before first tag lock; match TeleOp override when changed

    // --- REV Control Hub IMU physical mounting ---
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;      // Physical face of hub logo; adjust when remounted

    public static RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;    // Direction USB port points; keep consistent with LOGO_DIRECTION
}
