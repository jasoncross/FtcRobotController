/*
 * FILE: LauncherTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Collect every launcher-specific tuning value—RPM clamps, PIDF gains,
 *     encoder constants, and readiness tolerance—so TeleOp, Auto, and tests stay
 *     aligned without digging into the subsystem implementation.
 *   - Provide a single edit point alongside AutoRpmConfig for anything that
 *     affects flywheel performance.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Launcher speed & flywheel control)
 *   - FLYWHEEL_TPR
 *       • Encoder ticks per revolution at the wheel shaft. Multiply the motor’s
 *         ticks by any external gear ratio so RPM math stays correct.
 *   - RPM_MIN / RPM_MAX
 *       • Software clamps applied to every RPM request. Keep RPM_MAX ≥ far-shot
 *         demands from AutoRpmConfig and manual TeleOp ranges.
 *   - PID_P / PID_I / PID_D / PID_F
 *       • REV Hub velocity loop coefficients. Adjust after hardware changes or
 *         when you see overshoot/undershoot in telemetry.
 *   - AT_SPEED_TOLERANCE_RPM
 *       • Launcher-local readiness window used when callers do not specify one.
 *         Align with SharedRobotTuning.RPM_TOLERANCE so gating is consistent.
 */
package org.firstinspires.ftc.teamcode.config;

public final class LauncherTuning {
    private LauncherTuning() {}

    // Encoder and gearing constants
    public static double FLYWHEEL_TPR = 28.0;   // Ticks per revolution at the wheel shaft

    // Software RPM clamps
    public static double RPM_MIN = 0.0;         // Minimum allowed command
    public static double RPM_MAX = 6000.0;      // Maximum allowed command (keep ≥ AutoRpmConfig.FAR_RPM)

    // REV Hub RUN_USING_ENCODER PIDF coefficients
    public static double PID_P = 10.0;
    public static double PID_I = 3.0;
    public static double PID_D = 0.0;
    public static double PID_F = 12.0;

    // Default readiness tolerance when callers omit their own
    public static double AT_SPEED_TOLERANCE_RPM = 100.0;
}
