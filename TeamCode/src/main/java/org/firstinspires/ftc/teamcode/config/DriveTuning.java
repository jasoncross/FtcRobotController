/*
 * FILE: DriveTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Collect drivetrain geometry and control gains so encoder math and turn
 *     behavior stay consistent between TeleOp and Autonomous without editing the
 *     Drivebase implementation.
 *   - Provide a single stop for wheel size, gear ratio, strafing compensation,
 *     and IMU turn PID so field retunes happen quickly.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Drivetrain motion & positioning)
 *   - WHEEL_DIAMETER_IN / TICKS_PER_REV / GEAR_RATIO
 *       • Physical measurements required to convert encoder ticks ↔ inches.
 *         Update whenever wheels or cartridges change; GEAR_RATIO > 1 means the
 *         wheel turns slower than the motor.
 *   - STRAFE_CORRECTION
 *       • Empirical multiplier that compensates for mecanum lateral under-travel.
 *         Tune on your field until strafes land accurately.
 *   - TURN_KP / TURN_KD
 *       • IMU-based PD gains used during turn() helpers. Coordinate with
 *         SharedRobotTuning.TURN_TWIST_CAP so automation has the authority it
 *         expects.
 *   - TURN_TOLERANCE_DEG / TURN_SETTLE_TIME_SEC
 *       • Completion window and dwell time for IMU turns. Keep aligned with
 *         SharedRobotTuning.LOCK_TOLERANCE_DEG when autos rely on precise aim.
 *
 * CHANGES (2025-11-29): Added tunable translation speed floors for Auto move
 *                        tapering so distance accuracy stays configurable per
 *                        robot without editing Drivebase.java.
 */
package org.firstinspires.ftc.teamcode.config;

public final class DriveTuning {
    private DriveTuning() {}

    // Geometry & encoders
    public static double WHEEL_DIAMETER_IN = 4.098; // goBILDA 96 mm wheel ≈ 3.7795"
    public static double TICKS_PER_REV     = 537.7;  // goBILDA 5202 312 RPM output encoder
    public static double GEAR_RATIO        = 1.0;    // Wheel revs per motor rev (>1 if reduced)

    // Strafing compensation (empirical)
    public static double STRAFE_CORRECTION = 1.15;   // Multiply lateral component by this factor

    // IMU turn gains
    public static double TURN_KP = 0.012;            // Proportional gain for IMU turns
    public static double TURN_KD = 0.003;            // Derivative gain for IMU turns

    // Turn completion requirements
    public static double TURN_TOLERANCE_DEG   = 1.0;  // Acceptable heading error
    public static double TURN_SETTLE_TIME_SEC = 0.15; // Seconds inside tolerance before declaring done

    // Auto translation taper floors (prevent stalling near completion)
    public static double AUTO_MOVE_MIN_SPEED                 = 0.15; // Min speed for encoder-delta move() ramp
    public static double AUTO_MOVE_WITH_TWIST_MIN_TRANS_SPEED = 0.15; // Min translation speed for moveWithTwist() ramp
}
