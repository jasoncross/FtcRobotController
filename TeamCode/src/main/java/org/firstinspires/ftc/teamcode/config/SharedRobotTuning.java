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
 *   - RPM_TOLERANCE
 *       • Shared ±RPM window considered "at speed" by BaseAuto, AutoAimSpeed,
 *         and TeleOpAllianceBase when no override is provided.
 *       • Keep aligned with Launcher.atSpeedToleranceRPM if you tighten or loosen
 *         precision expectations.
 *   - RPM_READY_SETTLE_MS (ADDED 2025-11-03)
 *       • Minimum time the launcher must remain inside the RPM window before
 *         BaseAuto/TeleOp declare it "ready".
 *       • Keep modest so volleys remain responsive while filtering transient
 *         noise after large RPM adjustments.
 *   - HOLD_FIRE_FOR_RPM (ADDED 2026-01-03)
 *       • TeleOp-only control for whether feed engagement waits for RPM readiness.
 *       • ALL waits for every shot (including continuous holds), INITIAL waits only
 *         on the first shot/stream start, OFF disables the RPM gate.
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

    // CHANGES (2025-10-30): Moved INTAKE_ASSIST_MS to FeedTuning; kept deprecated alias for compatibility.
    // CHANGES (2025-11-02): Removed autonomous shot spacing tunable; cadence now provided per sequence.
    // CHANGES (2025-11-14): Added profile-specific lock tolerances so 480p vision can accept
    //                        higher bearing error without stalling volleys.
    // CHANGES (2025-11-24): Removed rotate-to-target timeout tuning; AutoSequence now passes
    //                        explicit per-step limits.
    // CHANGES (2026-01-03): Added HOLD_FIRE_FOR_RPM mode to control TeleOp RPM-ready feed gating.
    // --- REV Control Hub IMU physical mounting ---
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;      // Physical face of hub logo; adjust when remounted

    public static RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;    // Direction USB port points; keep consistent with LOGO_DIRECTION

    // --- Launcher speed gate ---
    public static double RPM_TOLERANCE              = 50.0;   // Shared ±RPM window; Launcher.atSpeedToleranceRPM should match
    public static long   RPM_READY_SETTLE_MS        = 150L;   // Time launcher must remain inside tolerance before declaring ready
    public enum HoldFireForRpmMode {
        ALL,
        INITIAL,
        OFF
    }
    public static HoldFireForRpmMode HOLD_FIRE_FOR_RPM = HoldFireForRpmMode.ALL; // TeleOp feed RPM gate: ALL=every shot, INITIAL=first only, OFF=disabled

    // --- Aim / drive caps used by Auto helpers (safe defaults) ---
    public static double LOCK_TOLERANCE_DEG         = 1.0;    // Bearing tolerance; keep aligned with Drivebase.TURN_TOLERANCE_DEG
    public static double LOCK_TOLERANCE_DEG_P480    = 1.5;    // Override when running the 640×480 vision profile (looser due to coarser pose output)
    public static double LOCK_TOLERANCE_DEG_P720    = 1.5;    // Override when running the 1280×720 profile (sharper pose accuracy)
    public static double TURN_TWIST_CAP             = 0.8;   // Twist clamp shared by BaseAuto + AutoAimSpeed unless overridden
    public static double DRIVE_MAX_POWER            = 1.0;   // Max auto drive power; adjust here for global movement speed

    // --- Assist behaviors shared across modes ---
    @Deprecated
    public static int    INTAKE_ASSIST_MS           = FeedTuning.INTAKE_ASSIST_MS; // 2025-10-30: moved to FeedTuning
    public static double INITIAL_AUTO_DEFAULT_SPEED = 2500.0; // Seed RPM before first tag lock; match TeleOp override when changed

}
