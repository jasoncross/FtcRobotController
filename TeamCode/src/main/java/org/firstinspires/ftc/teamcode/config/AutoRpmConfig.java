// ============================================================================
// FILE:           AutoRpmConfig.java
// LOCATION:       TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
//
// PURPOSE:
//   Single source of truth for the AutoSpeed (distance→RPM) mapping used by
//   BOTH TeleOp and Auto. Apply these tunables to any
//   LauncherAutoSpeedController with apply(...).
//
// NOTE ON HISTORY (WHERE THIS USED TO LIVE):
//   • TeleOpAllianceBase.java previously stored these as fields:
//       autoNearDistIn, autoNearRpm, autoFarDistIn, autoFarRpm, autoSmoothingAlpha
//   • BaseAuto.java relied on the controller's defaults.
//   Going forward, update them HERE.
//
// HOW TO USE:
//   AutoRpmConfig.apply(autoCtrl);
//
// AUTHOR: Indianola Robotics – 2025 Season (DECODE)
// ============================================================================
package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.control.LauncherAutoSpeedController;

public final class AutoRpmConfig {
    private AutoRpmConfig() {}

    // --- Tunables shared by TeleOp & Auto ---
    public static double NEAR_DIST_IN   = 24.0;
    public static double NEAR_RPM       = 1000.0;
    public static double FAR_DIST_IN    = 120.0;
    public static double FAR_RPM        = 4500.0;
    public static double SMOOTH_ALPHA   = 0.15;

    /** Apply standard params to a controller. Safe to call repeatedly. */
    public static void apply(LauncherAutoSpeedController ctrl) {
        if (ctrl == null) return;
        ctrl.setParams(NEAR_DIST_IN, NEAR_RPM, FAR_DIST_IN, FAR_RPM);
        ctrl.setSmoothingAlpha(SMOOTH_ALPHA);
    }
}
