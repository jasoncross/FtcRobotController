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
 * CHANGES (2025-12-29): Restored encoder ticks-per-rev to physical counts so
 *                        drive distances remain accurate for Auto moves.
 * CHANGES (2026-01-03): Added auto stall-exit detection tunables for encoder
 *                        moves so blocked drive steps can end early.
 * CHANGES (2026-01-18): Added cruise-then-taper and twist-bound tunables for
 *                        faster AutoSequence encoder moves with bounded heading hold.
 */
package org.firstinspires.ftc.teamcode.config;

public final class DriveTuning {
    private DriveTuning() {}

    // Geometry & encoders
    public static double WHEEL_DIAMETER_IN = 4.098; // goBILDA 96 mm wheel ≈ 3.7795"
    public static double TICKS_PER_REV     = 537.7;  // goBILDA 5202 312 RPM output encoder (counts per rev)
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
    public static double AUTO_MOVE_MIN_SPEED                 = 0.30; // Min speed for encoder-delta move() ramp
    public static double AUTO_MOVE_WITH_TWIST_MIN_TRANS_SPEED = 0.30; // Min translation speed for moveWithTwist() ramp
    public static double AUTO_MOVE_TAPER_START_FRACTION       = 0.10; // Fraction of distance (end window) to begin decel
    public static double AUTO_MOVE_TWIST_SCALE                = 0.60; // Scales auto heading-hold twist before mixing
    public static double AUTO_MOVE_MAX_TWIST                  = 0.35; // Absolute clamp on auto move twist power

    // Auto stall exit detection (Autonomous only)
    public static boolean AUTO_ENABLE_STALL_EXIT          = true;  // Allow auto encoder moves to exit early on stall
    public static double AUTO_STALL_VELOCITY_EPSILON       = 0.5;   // Inches/sec below which the drive is considered stalled
    public static double AUTO_STALL_POSITION_EPSILON       = 0.25;  // Inches of remaining error considered "not decreasing"
    public static double AUTO_STALL_TIME_MS                = 400.0; // Minimum time stalled before aborting the move
}
