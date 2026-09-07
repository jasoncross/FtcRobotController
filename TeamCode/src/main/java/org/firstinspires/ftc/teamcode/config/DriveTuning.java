/**
 * Retained DECODE encoder geometry, strafe correction, turn PD gains, taper and stall settings.
 * Review these historical values on the new robot before confirming calibration.
 * See docs/reusable-foundation.md for active settings and motion coordinate conventions.
 */
package org.firstinspires.ftc.teamcode.config;

public final class DriveTuning {
    private DriveTuning() {}

    // Retained DECODE baseline; enable only after review on the new chassis.
    public static boolean CALIBRATION_CONFIRMED = false;
    public static double AUTO_ACTION_TIMEOUT_SEC = 10.0;

    // Geometry & encoders
    public static double WHEEL_DIAMETER_IN = 4.098; // Historical effective diameter; not the nominal 96 mm wheel size
    public static double TICKS_PER_REV     = 537.7;  // goBILDA 5202 312 RPM output encoder (counts per rev)
    public static double GEAR_RATIO        = 1.0;    // Motor output revs per wheel rev (>1 for additional reduction)

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
