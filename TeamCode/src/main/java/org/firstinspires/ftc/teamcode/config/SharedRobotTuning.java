// ============================================================================
// FILE:           SharedRobotTuning.java
// LOCATION:       TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
//
// PURPOSE:
//   Centralize cross-mode (TeleOp + Auto) tunables that affect aiming, timing,
//   and “feel.” Update them here to keep both modes in sync.
//
// NOTE ON HISTORY (WHERE THESE USED TO LIVE):
//   • TeleOpAllianceBase.java:
//       - intakeAssistMs
//       - InitialAutoDefaultSpeed
//   • BaseAuto.java:
//       - SHOT_BETWEEN_MS
//       - RPM_TOLERANCE
//       - TURN_TWIST_CAP
//       - DRIVE_MAX_POWER
//
// AUTHOR: Indianola Robotics – 2025 Season (DECODE)
// ============================================================================
package org.firstinspires.ftc.teamcode.config;

public final class SharedRobotTuning {
    private SharedRobotTuning() {}

    // --- Shot timing ---
    public static long   SHOT_BETWEEN_MS            = 3000;   // 3 seconds between shots (all Auto modes)

    // --- Launcher speed gate ---
    public static double RPM_TOLERANCE              = 50.0;   // ± RPM for "at speed" gating (TeleOp + Auto)

    // --- Aim / drive caps used by Auto helpers (safe defaults) ---
    public static double LOCK_TOLERANCE_DEG         = 1.0;    // degrees off-center allowed for tag "lock"
    public static double TURN_TWIST_CAP             = 0.35;   // cap for aim/turn twist power
    public static double DRIVE_MAX_POWER            = 0.50;   // forward power for 24" move

    // --- Assist behaviors shared across modes ---
    public static int    INTAKE_ASSIST_MS           = 250;    // briefly run intake on feeds when it was OFF
    public static double INITIAL_AUTO_DEFAULT_SPEED = 2500.0; // RPM before first tag lock when AutoSpeed is enabled
}
