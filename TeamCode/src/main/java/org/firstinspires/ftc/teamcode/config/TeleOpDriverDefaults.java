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
 *   - INTAKE_ASSIST_MS
 *       • TeleOp copy of FeedTuning.INTAKE_ASSIST_MS; adjust here when diverging
 *         from the shared value.
 *   - INITIAL_AUTO_DEFAULT_SPEED
 *       • Local override of SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED for
 *         TeleOp-only experiments.
 */
package org.firstinspires.ftc.teamcode.config;

public final class TeleOpDriverDefaults {
    private TeleOpDriverDefaults() {}

    // CHANGES (2025-10-30): Point intake assist mirror at FeedTuning after tunable move.
    // CHANGES (2025-10-31): Default AutoSpeed + Intake to ON per driver request.
    // Startup toggles
    public static boolean AUTO_SPEED_ENABLED = true;
    public static boolean AUTO_AIM_ENABLED   = false;
    public static boolean INTAKE_ENABLED     = true;

    // Drive brake floor
    public static double SLOWEST_SPEED = 0.25;

    // Manual RPM slider bounds
    public static double RPM_BOTTOM = 0.0;
    public static double RPM_TOP    = 6000.0;

    // AutoAim grace + telemetry smoothing
    public static int    AUTO_AIM_LOSS_GRACE_MS = 4000;
    public static double TELEMETRY_SMOOTH_A     = 0.25;

    // RPM test adjustments (D-pad)
    public static double RPM_TEST_STEP = 50.0;

    // Intake assist + auto-speed seed (TeleOp overrides of shared values)
    public static int    INTAKE_ASSIST_MS           = FeedTuning.INTAKE_ASSIST_MS;
    public static double INITIAL_AUTO_DEFAULT_SPEED = SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED;

    // Auto-stop timer preferences
    public static boolean AUTO_STOP_TIMER_ENABLED = false;
    public static int     AUTO_STOP_TIMER_TIME_SEC = 119;
}
