# DECODE tuning reference

Exact configuration source from team checkout `69bb375` (February 17, 2026).
These are historical values, not BIOBUZZ calibration. See [reusable foundation](reusable-foundation.md) for the active settings.

## AutoAimTuning.java

```java
/*
 * FILE: AutoAimTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Provide optional overrides for AutoAimSpeed behavior without editing the
 *     assist class directly. These values default to the shared robot tuning so
 *     TeleOp and Auto stay aligned unless you explicitly diverge them here.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → AutoAim, targeting, and AprilTag alignment)
 *   - MAX_TWIST
 *       • Clamp applied to the twist recommendation returned by AutoAimSpeed.
 *         Defaults to SharedRobotTuning.TURN_TWIST_CAP; raise only if AutoAim
 *         needs to turn faster than BaseAuto helpers.
 *   - RPM_TOLERANCE
 *       • Readiness window used when checking launcher at-speed status. Matches
 *         SharedRobotTuning.RPM_TOLERANCE by default.
 *   - INITIAL_AUTO_DEFAULT_SPEED
 *       • Seed RPM applied before the first tag fix when AutoAim enables. Mirrors
 *         SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED unless overridden.
 *   - AUTO_AIM_SPEED_SCALE (NEW 2025-10-30)
 *       • Fractional drive speed applied to TeleOp translation while AutoAim is
 *         enabled. Lower to tighten alignment, raise toward 1.0 for faster
 *         approaches.
 *   - LONG_SHOT_ENABLED (NEW 2025-11-22)
 *       • Master switch for the alliance-biased long-shot lock window. Set false
 *         to keep the symmetric deadband even when beyond LONG_SHOT_DISTANCE_IN.
 *   - LONG_SHOT_DISTANCE_IN (NEW 2025-11-18)
 *       • Distance threshold (inches) above which AutoAim applies the
 *         alliance-biased lock window for long-range shots.
 *   - INVERT_TWIST (NEW 2026-01-08)
 *       • Optional sign flip applied once where twist feeds the drivebase so
 *         the physical "+rotation" direction matches TagAimController’s
 *         heading error convention (+right / -left).
 */
package org.firstinspires.ftc.teamcode.config;

public final class AutoAimTuning {
    private AutoAimTuning() {}

    // CHANGES (2025-11-22): Added LONG_SHOT_ENABLED master toggle to fall back to symmetric lock windows if needed.
    // CHANGES (2025-11-18): Added LONG_SHOT_DISTANCE_IN for alliance-biased long-range lock windows.
    // CHANGES (2025-10-30): Added AUTO_AIM_SPEED_SCALE for AutoAim drive throttling.
    public static double MAX_TWIST = SharedRobotTuning.TURN_TWIST_CAP;                // Twist clamp applied to aim corrections
    public static double RPM_TOLERANCE = SharedRobotTuning.RPM_TOLERANCE;             // Readiness window while AutoAim checks RPM
    public static double INITIAL_AUTO_DEFAULT_SPEED = SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED; // Seed RPM before first tag fix
    public static double AUTO_AIM_SPEED_SCALE = 0.25;                                 // Translation multiplier (0-1) while AutoAim is active
    public static boolean LONG_SHOT_ENABLED = true;                                    // Master toggle for alliance-biased long-shot lock behavior
    public static double LONG_SHOT_DISTANCE_IN = 110.0;                                // Range cutover (in) for long-shot lock biasing
    // Twist inversion now applies only to AutoAim (manual rotation is never flipped).
    public static boolean INVERT_TWIST = true;                                        // Legacy flag (kept for compatibility)
    public static boolean INVERT_AIM_TWIST = INVERT_TWIST;                             // Flip AutoAim twist only
}

```

## AutoRpmConfig.java

```java
/*
 * FILE: AutoRpmConfig.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Store the authoritative distance→RPM anchor points and smoothing factor
 *     for LauncherAutoSpeedController so TeleOp and Autonomous read the exact
 *     same curve each time they initialize.
 *   - Provide a single apply(...) helper invoked from BaseAuto and
 *     TeleOpAllianceBase to copy the values into the controller at runtime.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Launcher speed & flywheel control)
 *   - BLUE_CALIBRATION_DISTANCES_IN / BLUE_CALIBRATION_RPMS (ADDED 2025-11-15, SPLIT 2026-01-17)
 *       • Ordered arrays of inches + RPM pairs that define the BLUE alliance calibration table.
 *       • Supports any N ≥ 2 entries. LauncherAutoSpeedController interpolates between
 *         neighboring points and clamps outside the table bounds.
 *   - RED_CALIBRATION_DISTANCES_IN / RED_CALIBRATION_RPMS (ADDED 2026-01-17)
 *       • Ordered arrays of inches + RPM pairs that define the RED alliance calibration table.
 *       • Supports any N ≥ 2 entries. LauncherAutoSpeedController interpolates between
 *         neighboring points and clamps outside the table bounds.
 *   - SMOOTH_ALPHA
 *       • Exponential smoothing constant (0–1) applied to RPM updates.
 *       • Overrides LauncherAutoSpeedController.smoothingAlpha; match the value
 *         here to what TunableDirectory recommends so TeleOp lab tests mirror
 *         match play.
 *   - DEFAULT_NO_TAG_RPM (ADDED 2025-10-31, REMOVED 2026-01-03)
 *       • RPM commanded whenever AutoSpeed is active but no AprilTag is locked yet.
 *         Now derived from the farthest calibration point instead of a manual value.
 *
 * METHODS
 *   - apply(LauncherAutoSpeedController ctrl, Alliance alliance)
 *       • Copies all tunables into the supplied controller for the active alliance. Call from TeleOp init
 *         and BaseAuto runOpMode() before relying on AutoSpeed.
 *
 * NOTES
 *   - AutoAimSpeed and BaseAuto both read the controller after apply(), so this
 *     file is the single source of truth for curve anchors.
 *   - Provide at least two calibration points; values do not need to be evenly spaced.
 *   - Launcher.RPM_MIN/RPM_MAX still clamp the final command; adjust those in
 *     subsystems/Launcher.java when hardware changes require broader limits.
 *
 * CHANGES (2026-01-17): Split AutoRPM calibration tables by alliance and
 *                       applied the selection through AutoRpmConfig.apply(...).
 */
package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.control.LauncherAutoSpeedController;

public final class AutoRpmConfig {
    private AutoRpmConfig() {}

    // --- Tunables shared by TeleOp & Auto ---
    // CHANGES (2025-11-15): Replaced fixed near/far anchors with a full calibration table (default 6 points, 35–100 in).
    // CHANGES (2026-01-03): Default no-tag RPM now derives from the farthest calibration point.
    // CHANGES (2026-01-17): Split AutoRPM calibration tables by alliance (RED/BLUE) with identical defaults.
    public static double[] BLUE_CALIBRATION_DISTANCES_IN = {
            48.0,
            55.0,
            65.0,
            78.0,
            88.0,
            115.0,
            120.0,
            130.0,
            140.0
    }; // Blue alliance inches for the calibration table (must align with BLUE_CALIBRATION_RPMS)
    public static double[] BLUE_CALIBRATION_RPMS = {
            2350.0,
            2550.0,
            2550.0,
            2700.0,
            2900.0,
            3800.0,
            3900.0,
            4050.0,
            4300.0
    }; // Blue alliance RPM values paired with BLUE_CALIBRATION_DISTANCES_IN entries
    public static double[] RED_CALIBRATION_DISTANCES_IN = {
            48.0,
            55.0,
            65.0,
            78.0,
            88.0,
            115.0,
            120.0,
            130.0,
            140.0
    }; // Red alliance inches for the calibration table (must align with RED_CALIBRATION_RPMS)
    public static double[] RED_CALIBRATION_RPMS = {
            2350.0,
            2550.0,
            2550.0,
            2700.0,
            2900.0,
            3800.0,
            3900.0,
            4050.0,
            4300.0
    }; // Red alliance RPM values paired with RED_CALIBRATION_DISTANCES_IN entries
    public static double SMOOTH_ALPHA      = 0.15;  // Exponential smoothing factor applied after every apply()

    /** Apply standard params to a controller for the requested alliance. Safe to call repeatedly. */
    public static void apply(LauncherAutoSpeedController ctrl, Alliance alliance) {
        if (ctrl == null) return;
        ctrl.setDefaultRpm(resolveNoTagRpm(alliance));
        ctrl.setCalibrationCurve(resolveDistances(alliance), resolveRpms(alliance));
        ctrl.setSmoothingAlpha(SMOOTH_ALPHA);
    }

    /** RPM to hold while AutoSpeed runs without a tag lock (uses farthest calibration point). */
    public static double resolveNoTagRpm(Alliance alliance) {
        double[] distances = resolveDistances(alliance);
        double[] rpms = resolveRpms(alliance);
        if (distances == null || rpms == null) return 0.0;
        if (distances.length == 0 || rpms.length == 0) return 0.0;
        int maxIndex = 0;
        double maxDistance = distances[0];
        int count = Math.min(distances.length, rpms.length);
        for (int i = 1; i < count; i++) {
            if (distances[i] > maxDistance) {
                maxDistance = distances[i];
                maxIndex = i;
            }
        }
        return rpms[maxIndex];
    }

    private static double[] resolveDistances(Alliance alliance) {
        return (alliance == Alliance.RED) ? RED_CALIBRATION_DISTANCES_IN : BLUE_CALIBRATION_DISTANCES_IN;
    }

    private static double[] resolveRpms(Alliance alliance) {
        return (alliance == Alliance.RED) ? RED_CALIBRATION_RPMS : BLUE_CALIBRATION_RPMS;
    }
}

```

## ControllerTuning.java

```java
/*
 * FILE: ControllerTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Expose controller-level thresholds that affect how trigger presses are
 *     interpreted so students can tweak input feel without modifying the binding
 *     system.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Controller interface & misc utilities)
 *   - TRIGGER_EDGE_THRESH
 *       • Analog trigger value treated as a "pressed" button when using
 *         bindTriggerPress. Lower for lighter pulls; raise when drivers bump
 *         triggers accidentally.
 */
package org.firstinspires.ftc.teamcode.config;

public final class ControllerTuning {
    private ControllerTuning() {}

    public static double TRIGGER_EDGE_THRESH = 0.5; // Trigger value treated as a press when edge-detected
}

```

## DebugTelemetryConfig.java

```java
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

```

## DriveTuning.java

```java
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

```

## FeedStopConfig.java

```java
/*
 * FILE: FeedStopConfig.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Centralize FeedStop servo tuning so TeleOp and Autonomous share the same
 *     travel limits, block/release setpoints, and release timing.
 *   - Allow on-robot adjustments to narrow the PWM range for faster movement
 *     while keeping code free of hard-coded numbers.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Shot cadence, feed, and eject)
 *   - USE_AUTO_SCALE
 *       • When true, Feed computes a ServoImplEx.scaleRange() window that just
 *         fits the requested hold/release angles plus a safety margin. When
 *         false (default) the servo runs its full 300° span with no scaling.
 *   - DIRECTION_SIGN
 *       • Keeps the logical positive direction aligned with physical RELEASE
 *         motion so teams can flip behavior without rewiring.
 *   - SAFE_PRESET_OPEN_DEG / MAX_HOME_TRAVEL_DEG / HOME_BACKOFF_DEG / HOME_DWELL_MS
 *       • The guarded homing approach: how far to open (CW) before reversing
 *         toward the stop, how far the safe sweep may travel before aborting,
 *         how far to back away after seating, and how long to dwell on the
 *         hard stop to establish a repeatable zero.
 *   - SKIP_SAFE_OPEN_ON_START
 *       • When true, the homing routine skips the safe-open pre-sweep and
 *         goes straight to the blocking stop after START to avoid gate twitch.
 *   - HOLD_ANGLE_DEG / RELEASE_ANGLE_DEG
 *       • Target angles (relative to zero) used while blocking the feed path
 *         and fully releasing for a shot.
 *   - SOFT_CCW_LIMIT_DEG / SOFT_CW_LIMIT_DEG
 *       • Software clamps that prevent commands below 0° or above the safe
 *         release sweep; every request is clamped into this band.
 *   - SAFETY_MARGIN_DEG
 *       • Extra clearance applied when auto-expanding the servo scale window so
 *         HOLD/RELEASE angles never demand the full PWM span.
 *   - RELEASE_HOLD_MS
 *       • How long the servo stays at RELEASE after a fire request before
 *         snapping back to BLOCK.
 *   - FIRE_LEAD_MS
 *       • Lead time between requesting RELEASE and starting the feed motor so
 *         the gate clears before the feed wheel pushes.
 */
package org.firstinspires.ftc.teamcode.config;

public final class FeedStopConfig {
    private FeedStopConfig() {}

    // CHANGES (2025-11-06): Initial FeedStop servo tuning defaults for goBILDA 25-3 speed servo.
    // CHANGES (2025-11-07): Added homing + degrees-based control tunables (overshoot, backoff, release angle, direction sign).
    // CHANGES (2025-11-08): Split HOLD vs. RELEASE degree targets, added safety margin auto-scaling, and removed the fixed
    //                       degrees-per-unit constant in favor of runtime span math.
    // CHANGES (2025-11-09): Default to full-span servo travel, added USE_AUTO_SCALE toggle, and retired SCALE_MIN/MAX tunables.
    // CHANGES (2025-11-07): Added guarded homing presets (safe open + travel cap) and soft-limit clamps for degrees-based control.
    // CHANGES (2025-12-31): Added SKIP_SAFE_OPEN_ON_START to prevent a brief gate dip at START.
    // CHANGES (2026-01-07): Added anti-jitter command guard tunables for FeedStop servo updates.
    public static boolean USE_AUTO_SCALE = false;    // Optional servo auto-scaling toggle (default full span)
    public static double DIRECTION_SIGN = +1.0;      // +1 when release is toward higher PWM, -1 when toward lower PWM
    public static boolean SKIP_SAFE_OPEN_ON_START = true; // Skip the safe-open sweep when homing after START
    public static double SAFE_PRESET_OPEN_DEG = 40.0; // Target angle used during the homing safe-open approach (deg)
    public static double HOME_OVERSHOOT_DEG = 12.0;  // Legacy alias for SAFE_PRESET_OPEN_DEG (kept for backward compatibility)
    public static double MAX_HOME_TRAVEL_DEG = 120.0; // Maximum degrees allowed during homing safe-open sweep (deg)
    public static double HOME_BACKOFF_DEG = 2.0;     // Distance (deg) to back away from the hard stop after seating
    public static long HOME_DWELL_MS = 120;          // Time (ms) to dwell against the BLOCK stop before backoff
    public static double SOFT_CCW_LIMIT_DEG = 0.0;    // Lowest allowed command (deg relative to home)
    public static double SOFT_CW_LIMIT_DEG = 170.0;   // Highest allowed command (deg relative to home)
    public static double HOLD_ANGLE_DEG = 5.0;     // Hold (blocking) angle relative to the homed zero position (deg)
    public static double RELEASE_ANGLE_DEG = 75.0; // Release angle relative to the homed zero position (deg)
    public static double SAFETY_MARGIN_DEG = 5.0;   // Extra clearance added when sizing the PWM span (deg)
    public static long RELEASE_HOLD_MS = 250;       // Duration to remain at RELEASE (ms)
    public static long FIRE_LEAD_MS = 500;           // Delay before feed motor starts (ms)
    public static double COMMAND_EPSILON_DEG = 0.35; // FeedStop command deadband; smaller deltas skip servo updates (deg)
    public static long COMMAND_MIN_PERIOD_MS = 250;  // Minimum time between identical FeedStop commands (ms)
}

```

## FeedTuning.java

```java
/*
 * FILE: FeedTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Own the shared feed motor parameters used by both TeleOp and Autonomous so
 *     cadence tweaks happen in one place instead of inside Feed.java.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Shot cadence, feed, and eject)
 *   - FIRE_POWER
 *       • Motor power applied while the feed pulse runs. Increase toward 1.0 when
 *         artifacts stick; drop toward 0.7 if jams become frequent.
 *   - FIRE_TIME_MS
 *       • Timeout used when shot detection fails; feed stops when encoder drop
 *         or this timeout is reached.
 *   - MIN_CYCLE_MS
 *       • Legacy cooldown between feeds; still referenced by some Auto routines.
 *   - FIRE_POWER_LAUNCHING
 *       • Reduced feed power after shot detection to prevent shove.
 *   - FIRING_DROP_RPM_THRESHOLD
 *       • RPM drop from target that indicates a ball has entered the launcher.
 *   - INTAKE_ASSIST_MS (MOVED 2025-10-30)
 *       • Duration TeleOp/Auto run the intake after a feed when it was previously
 *         off. Centralized here alongside other feed cadence values.
 *   - IDLE_HOLD_POWER (ADDED 2025-10-31)
 *       • Counter-rotation power applied while idle to keep the feed staged.
 *         Set to 0 to fall back to BRAKE-only holding.
 */
package org.firstinspires.ftc.teamcode.config;

public final class FeedTuning {
    private FeedTuning() {}

    // CHANGES (2025-10-30): Added INTAKE_ASSIST_MS after moving ownership from SharedRobotTuning.
    // CHANGES (2025-10-31): Added IDLE_HOLD_POWER to keep the feed counter-rotating while idle.
    // CHANGES (2025-10-31): Increased IDLE_HOLD_POWER magnitude so staged artifacts remain latched.
    // CHANGES (2025-11-02): Updated documentation to reflect per-sequence shot cadence control.
    // CHANGES (2026-01-04): Added RPM drop detection + post-shot feed power controls for shared firing.
    public static double FIRE_POWER = 0.8;          // Motor power applied during each feed pulse
    public static int FIRE_TIME_MS  = 1300;         // Timeout (ms) before assuming shot fired when no RPM drop is detected
    public static int MIN_CYCLE_MS  = 1000;         // Legacy spacing (ms) between successive feed pulses (Auto only)
    public static double FIRE_POWER_LAUNCHING = 0.10; // Reduced feed power after shot detection to prevent shove
    public static double FIRING_DROP_RPM_THRESHOLD = 600; // RPM drop from target to declare a shot captured
    public static int INTAKE_ASSIST_MS = 250; // Intake assist duration (ms) after a feed when intake was off
    public static double IDLE_HOLD_POWER = -0.1; // Idle counter-rotation power (0 = BRAKE only)
}

```

## IntakeTuning.java

```java
/*
 * FILE: IntakeTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Centralize every intake power/encoder threshold so TeleOp and Auto stay aligned
 *     as the hardware is retuned for the DECODE season ball column.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Intake power & driver defaults)
 *   - fillPower
 *       • Motor power while the shaft is freely filling (default mirrors the historical
 *         POWER_ON constant).
 *   - packingPower
 *       • Slightly reduced power once the first ball contacts the feed column so we can
 *         compress the stack without max current draw.
 *   - holdPower
 *       • Low power used during the SATURATED hold pulse to maintain positive pressure
 *         on the packed stack.
 *   - holdPulsePeriodMs / holdPulseOnMs
 *       • Defines the duty cycle for the SATURATED hold pulse. Set period ≤ 0 to run a
 *         steady holdPower instead of pulsing.
 *   - feedActiveHoldPower
 *       • Alternate hold power asserted whenever the Feed cycle is active so we are not
 *         adding load while launching.
 *   - sampleIntervalMs
 *       • Encoder sample cadence for classifying motion. Lower intervals react faster
 *         but increase noise.
 *   - freeDeltaTicks / contactDeltaTicks / stallDeltaTicks
 *       • Movement thresholds (ticks per sample) used to categorize "free flow",
 *         "packing", and "stall" behavior.
 *   - packingRangeTicks
 *       • Encoder travel allowed after first contact before the column is treated as
 *         fully packed (≈3 balls).
 *   - stallDebounceSamples
 *       • Number of consecutive stall-level samples required before declaring a true jam.
 *   - jamRecoveryPauseMs
 *       • Cooldown window applied after a jam before retrying forward flow.
 */
package org.firstinspires.ftc.teamcode.config;

public final class IntakeTuning {
    private IntakeTuning() {}

    // CHANGES (2025-11-16): Added inline comments for each tunable constant.
    // CHANGES (2025-11-25): Added reverse pulse power/duration for the triple-tap RB gesture.
    // CHANGES (2025-11-25): Removed the timed reverse duration in favor of a latched
    //                       reverse mode that exits on the next intake toggle tap and aligned
    //                       changelog dates with the 2025-11-25 release.

    public static double FILL_POWER = 0.9; // Motor power used while the intake is freely filling
    public static double PACKING_POWER = 0.75; // Reduced power once first contact occurs to compress the column
    public static double HOLD_POWER = 0.35; // Base power applied during the SATURATED hold pulse
    public static double FEED_ACTIVE_HOLD_POWER = 0.75; // Alternate hold power while the feed subsystem is cycling
    public static double REVERSE_POWER = -0.6; // Power applied during the latched reverse run to clear jams

    public static int HOLD_PULSE_PERIOD_MS = 400; // Total period (ms) for the SATURATED hold pulse cadence
    public static int HOLD_PULSE_ON_MS = 120; // Portion of the pulse period (ms) spent applying HOLD_POWER

    public static int SAMPLE_INTERVAL_MS = 50; // Encoder sampling cadence (ms) for movement classification
    public static int FREE_DELTA_TICKS = 20; // Minimum tick delta per sample to qualify as free-flow motion
    public static int CONTACT_DELTA_TICKS = 5; // Tick delta threshold indicating initial contact/packing behavior
    public static int STALL_DELTA_TICKS = 2; // Tick delta treated as a true stall when sustained
    public static int PACKING_RANGE_TICKS = 650; // Encoder travel allowed after first contact before saturation
    public static int STALL_DEBOUNCE_SAMPLES = 8; // Number of consecutive stall samples required before JAMMED state
    public static int JAM_RECOVERY_PAUSE_MS = 250; // Motor cooldown duration (ms) after declaring a jam
}

```

## LauncherTuning.java

```java
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
 *       • Software clamps applied to every RPM request. Keep RPM_MAX ≥ the
 *         highest AutoRpmConfig calibration RPM and manual TeleOp ranges.
 *   - PID_P / PID_I / PID_D / PID_F
 *       • REV Hub velocity loop coefficients. Adjust after hardware changes or
 *         when you see overshoot/undershoot in telemetry.
 *   - AT_SPEED_TOLERANCE_RPM
 *       • Launcher-local readiness window used when callers do not specify one.
 *         Align with SharedRobotTuning.RPM_TOLERANCE so gating is consistent.
 *   - MANUAL_RPM_STEP (NEW 2025-10-30)
 *       • D-pad increment used when TeleOp manual mode nudges RPM (AutoSpeed off with Manual Lock engaged).
 *         Match RPM test adjustments if you want identical feel between modes.
 */
package org.firstinspires.ftc.teamcode.config;

public final class LauncherTuning {
    private LauncherTuning() {}

    // CHANGES (2025-10-30): Added MANUAL_RPM_STEP for TeleOp manual D-pad nudges.
    // Encoder and gearing constants
    public static double FLYWHEEL_TPR = 28.0;   // Ticks per revolution at the wheel shaft

    // Software RPM clamps
    // CHANGES (2025-11-15): Clarified RPM_MAX guidance now that AutoRpmConfig exposes a full calibration table.
    public static double RPM_MIN = 2000.0;         // Minimum allowed command
    public static double RPM_MAX = 5000.0;      // Maximum allowed command (keep ≥ highest AutoRpmConfig calibration RPM)

    // REV Hub RUN_USING_ENCODER PIDF coefficients
    public static double PID_P = 10.0;              // Proportional gain for REV velocity loop
    public static double PID_I = 3.0;               // Integral gain for REV velocity loop
    public static double PID_D = 0.0;               // Derivative gain for REV velocity loop
    public static double PID_F = 12.0;              // Feedforward coefficient for REV velocity loop

    // Default readiness tolerance when callers omit their own
    public static double AT_SPEED_TOLERANCE_RPM = 100.0; // RPM error allowed when checking readiness locally
    public static double MANUAL_RPM_STEP = 50.0; // RPM step per D-pad press (AutoSpeed off & manual lock ON)
}

```

## LimelightPipelineAutoSelectConfig.java

```java
package org.firstinspires.ftc.teamcode.config;

/*
 * FILE: LimelightPipelineAutoSelectConfig.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Centralize Limelight pipeline auto-selection tunables so TeleOp and Auto
 *     share the same INIT-time evaluation behavior without code edits.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Vision & range calibration)
 *   - ENABLE_LIMELIGHT_PIPELINE_AUTOSELECT
 *       • Master switch for INIT pipeline auto-selection in TeleOp + Auto.
 *   - LIMELIGHT_PIPELINE_PROFILES
 *       • Ordered list of pipelineIndex:description entries to evaluate.
 *         Format: "0:Meet Gym;1:Practice Field;2:Practice Field (Dim)".
 *   - PIPELINE_SETTLE_MS
 *       • Time to wait after switching pipelines before sampling frames (ms).
 *   - PIPELINE_SAMPLE_COUNT
 *       • Number of frames to sample per pipeline before scoring.
 *   - PIPELINE_SAMPLE_INTERVAL_MS
 *       • Delay between samples while scoring a pipeline (ms).
 *   - PIPELINE_MAX_SELECTION_MS
 *       • Max time allowed for selection before timing out (ms).
 *   - PIPELINE_MIN_GOAL_HITS
 *       • Minimum goal detections required to qualify a pipeline.
 *   - PIPELINE_MIN_OPP_GOAL_HITS
 *       • Minimum opposing goal detections required to qualify a pipeline.
 *   - PIPELINE_REQUIRE_GOAL_ONLY
 *       • If true, ignore opposing goal detections for qualification.
 *   - PIPELINE_FALLBACK_INDEX
 *       • Pipeline index to use when selection fails or times out.
 *   - PIPELINE_REMEMBER_LAST_GOOD_ENABLED
 *       • Enable storing and using the last successful pipeline as a fallback.
 *   - PIPELINE_PERSIST_LAST_GOOD_ENABLED
 *       • If true, persist the last-good pipeline to SharedPreferences.
 *   - PIPELINE_USE_LAST_GOOD_ON_TIMEOUT
 *       • If true, prefer last-good pipeline when selection times out.
 *
 * CHANGES (2025-12-28): Added Limelight pipeline auto-selection tunables for
 *                       INIT-time profile evaluation and telemetry.
 * CHANGES (2025-12-28): Added hit-count thresholds and an optional goal-only
 *                       mode for stable pipeline qualification.
 * CHANGES (2025-12-28): Added a tunable fallback pipeline index instead of
 *                       hard-coding pipeline 0.
 * CHANGES (2025-12-28): Added tunables to remember the last successful
 *                       pipeline and optionally persist it for fallback use.
 */
public final class LimelightPipelineAutoSelectConfig {
    private LimelightPipelineAutoSelectConfig() {}

    public static boolean ENABLE_LIMELIGHT_PIPELINE_AUTOSELECT = true; // Enable INIT auto-selection (TeleOp + Auto)
    public static String LIMELIGHT_PIPELINE_PROFILES = "0:Meet Gym;1:Practice Field;2:Practice Field (Dim)"; // Ordered pipeline profiles to test
    public static long PIPELINE_SETTLE_MS = 250L; // Settling time after pipeline switch (ms)
    public static int PIPELINE_SAMPLE_COUNT = 6; // Frames sampled per pipeline
    public static long PIPELINE_SAMPLE_INTERVAL_MS = 60L; // Delay between samples (ms)
    public static long PIPELINE_MAX_SELECTION_MS = 5000L; // Max total selection time (ms)
    public static int PIPELINE_MIN_GOAL_HITS = 5; // Minimum goal detections to qualify a pipeline
    public static int PIPELINE_MIN_OPP_GOAL_HITS = 2; // Minimum opposing goal detections to qualify a pipeline
    public static boolean PIPELINE_REQUIRE_GOAL_ONLY = false; // If true, ignore opposing goal detections for ranking
    public static int PIPELINE_FALLBACK_INDEX = 0; // Pipeline index used when auto-selection fails or times out
    public static boolean PIPELINE_REMEMBER_LAST_GOOD_ENABLED = true; // Use last successful pipeline when no tags are seen
    public static boolean PIPELINE_PERSIST_LAST_GOOD_ENABLED = true; // Persist last-good pipeline to SharedPreferences
    public static boolean PIPELINE_USE_LAST_GOOD_ON_TIMEOUT = true; // Allow last-good fallback on selection timeout
}

```

## OdometryConfig.java

```java
package org.firstinspires.ftc.teamcode.config;

/*
 * FILE: OdometryConfig.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Centralize all odometry + field layout tunables so the fused pose, camera
 *     offsets, and Dashboard drawings stay aligned across TeleOp and Auto.
 *   - Coordinates follow the DECODE field convention: (0,0) at the center of
 *     the field, +X to the right when facing the targets, +Y toward the target
 *     wall. All Y values below are already expressed in this centered frame.
 *
 * NOTES
 *   - Update these values with real measurements from the competition robot and
 *     practice field; every constant includes units and an inline description
 *     for quick pit-side edits.
 *
 * CHANGES (2025-11-26): Switched launch zones to triangle vertices, replaced
 *                        artifact row anchors with alliance-aware row starts,
 *                        spacing, and radius, and documented the +X/+Y field
 *                        frame used by odometry + dashboard drawing.
 * CHANGES (2025-11-25): Added goal AprilTag pose tunables (XYZ + yaw) to support
 *                        tag-based odometry fusion with camera offsets.
 * CHANGES (2025-12-11): Recentered all field geometry to the FTC-standard
 *                        field-center origin (+Y toward targets, +X right)
 *                        ahead of Limelight-only position fusion.
 * CHANGES (2025-12-29): Added a dedicated odometry distance scale so pose
 *                        calibration no longer affects Auto drive distances.
 */
public final class OdometryConfig {

    // ==== Coordinate system & robot geometry (inches) ====
    public static final double FIELD_LENGTH = 144.0;        // inches, Y span from human wall to target wall
    public static final double FIELD_WIDTH  = 144.0;        // inches, X span from left to right wall
    public static final double HUMAN_WALL_Y = -FIELD_LENGTH / 2.0;          // Y position of the human wall (centered frame)
    public static final double TARGET_WALL_Y = FIELD_LENGTH / 2.0;       // Y position of the target wall (centered frame)
    public static final double FIELD_CENTER_X = 0.0;        // X origin at field centerline
    public static final double LEFT_FIELD_X  = -72.0;       // Left field edge (human perspective)
    public static final double RIGHT_FIELD_X = 72.0;        // Right field edge

    public static final double ROBOT_HALF_LENGTH = 9.0;     // Half-length of robot footprint, inches
    public static final double ROBOT_HALF_WIDTH  = 7.875;     // Half-width of robot footprint, inches

    public static final double INTAKE_OFFSET_X = 0.0;      // Intake center offset right (+X) from robot center, inches
    public static final double INTAKE_OFFSET_Y = -9.0;       // Intake center offset forward (+Y) from robot center, inches

    public static final double LAUNCHER_OFFSET_X = 0.0;    // Launcher exit offset right (+X) from robot center, inches
    public static final double LAUNCHER_OFFSET_Y = 9.0;     // Launcher exit offset forward (+Y) from robot center, inches

    public static final double CAMERA_OFFSET_X = 0.0;       // Camera offset right (+X) from robot center, inches
    public static final double CAMERA_OFFSET_Y = 8.75;       // Camera offset forward (+Y) from robot center, inches
    public static final double CAMERA_OFFSET_Z = 10.25;      // Camera height above floor, inches
    public static final double CAMERA_PITCH_DEG = 14.0;    // Camera pitch downward (-) relative to robot frame, degrees
    public static final double CAMERA_YAW_DEG   = 0.0;      // Camera yaw relative to robot heading, degrees
    public static final double ODOMETRY_DISTANCE_SCALE = 4.0; // Scale factor for odometry-only wheel deltas (unitless)

    // ==== AprilTag goal poses (inches/degrees) ====
    public static final double TAG_RED_GOAL_X       = 56.5;   // Red goal AprilTag center X, inches
    public static final double TAG_RED_GOAL_Y       = 58.0;   // Red goal AprilTag center Y, inches
    public static final double TAG_RED_GOAL_Z       = 29.5;    // Red goal AprilTag center height above floor, inches
    public static final double TAG_RED_GOAL_YAW_DEG = 125.0;   // Red goal AprilTag yaw facing field, degrees

    public static final double TAG_BLUE_GOAL_X       = -56.5;   // Blue goal AprilTag center X, inches
    public static final double TAG_BLUE_GOAL_Y       = 58.0;  // Blue goal AprilTag center Y, inches
    public static final double TAG_BLUE_GOAL_Z       = 29.5;   // Blue goal AprilTag center height above floor, inches
    public static final double TAG_BLUE_GOAL_YAW_DEG = 235.0;  // Blue goal AprilTag yaw facing field, degrees

    // ==== Field elements – goals & classifiers (inches) ====
    public static final double GOAL_RED_X  = 60.0;         // Red goal tag center X
    public static final double GOAL_RED_Y  = 72.0;         // Red goal tag center Y
    public static final double GOAL_BLUE_X = -60.0;          // Blue goal tag center X
    public static final double GOAL_BLUE_Y = 72.0;         // Blue goal tag center Y

    public static final double CLASSIFIER_RED_X  = 68.5;   // Red classifier center X
    public static final double CLASSIFIER_RED_Y  = 0.0;   // Red classifier center Y
    public static final double CLASSIFIER_BLUE_X = -68.5;    // Blue classifier center X
    public static final double CLASSIFIER_BLUE_Y = 0.0;   // Blue classifier center Y

    // ==== Gates, gate zones, secret tunnel (inches) FIX THESE ALONG WITH GATE ZONE ====
    public static final double GATE_RED_X  = -60.0;         // Red gate bar center X
    public static final double GATE_RED_Y  = 60.0;         // Red gate bar center Y
    public static final double GATE_BLUE_X = 60.0;          // Blue gate bar center X
    public static final double GATE_BLUE_Y = 60.0;         // Blue gate bar center Y

    public static final double GATE_ZONE_RED_CENTER_X  = 54.0;  // Red gate zone center X
    public static final double GATE_ZONE_RED_CENTER_Y  = 0.0;  // Red gate zone center Y
    public static final double GATE_ZONE_BLUE_CENTER_X = -54.0;   // Blue gate zone center X
    public static final double GATE_ZONE_BLUE_CENTER_Y = 0.0;  // Blue gate zone center Y
    public static final double GATE_ZONE_WIDTH = 12.0;           // Gate zone width, inches
    public static final double GATE_ZONE_DEPTH = 3.0;           // Gate zone depth, inches

    public static final double SECRET_TUNNEL_RED_X  = -68.5;     // Red wall secret tunnel X
    public static final double SECRET_TUNNEL_RED_Y1 = -49.25;      // Red tunnel start Y
    public static final double SECRET_TUNNEL_RED_Y2 = -2.5;      // Red tunnel end Y
    public static final double SECRET_TUNNEL_BLUE_X  = 68.5;     // Blue wall secret tunnel X
    public static final double SECRET_TUNNEL_BLUE_Y1 = -49.25;     // Blue tunnel start Y
    public static final double SECRET_TUNNEL_BLUE_Y2 = -2.5;     // Blue tunnel end Y

    // ==== Loading zones & base zones (inches) ====
    public static final double LOADING_ZONE_RED_CENTER_X  = 60.5; // Red loading zone center X
    public static final double LOADING_ZONE_RED_CENTER_Y  = -60.5;  // Red loading zone center Y
    public static final double LOADING_ZONE_BLUE_CENTER_X = -60.5;  // Blue loading zone center X
    public static final double LOADING_ZONE_BLUE_CENTER_Y = -60.5;  // Blue loading zone center Y
    public static final double LOADING_ZONE_SIZE = 23.0;           // Loading zone square size, inches

    public static final double BASE_ZONE_RED_CENTER_X  = -33.5;    // Red base zone center X
    public static final double BASE_ZONE_RED_CENTER_Y  = -38.5;     // Red base zone center Y
    public static final double BASE_ZONE_BLUE_CENTER_X = 33.5;     // Blue base zone center X
    public static final double BASE_ZONE_BLUE_CENTER_Y = -38.5;     // Blue base zone center Y
    public static final double BASE_ZONE_SIZE = 18.0;              // Base zone square size, inches

    // ==== Launch zones & lines (inches) ====
    public static final double HUMAN_LAUNCH_LEFT_X = -24.0;        // Human-side launch triangle left base X (touching wall)
    public static final double HUMAN_LAUNCH_LEFT_Y = -72.0;          // Human-side launch triangle left base Y
    public static final double HUMAN_LAUNCH_RIGHT_X = 24.0;        // Human-side launch triangle right base X
    public static final double HUMAN_LAUNCH_RIGHT_Y = -72.0;         // Human-side launch triangle right base Y
    public static final double HUMAN_LAUNCH_APEX_X = 0.0;          // Human-side launch triangle apex X (upfield point)
    public static final double HUMAN_LAUNCH_APEX_Y = -48.0;         // Human-side launch triangle apex Y

    public static final double TARGET_LAUNCH_LEFT_X = -72.0;       // Target-side launch triangle left base X (touching wall)
    public static final double TARGET_LAUNCH_LEFT_Y = 72.0;       // Target-side launch triangle left base Y
    public static final double TARGET_LAUNCH_RIGHT_X = 72.0;       // Target-side launch triangle right base X
    public static final double TARGET_LAUNCH_RIGHT_Y = 72.0;      // Target-side launch triangle right base Y
    public static final double TARGET_LAUNCH_APEX_X = 0.0;         // Target-side launch triangle apex X (back toward center)
    public static final double TARGET_LAUNCH_APEX_Y = 0.0;        // Target-side launch triangle apex Y

    public static final double HUMAN_LAUNCH_LINE_Y  = -48.0;        // Y coordinate of human launch line (across triangle base)
    public static final double TARGET_LAUNCH_LINE_Y = 48.0;       // Y coordinate of target launch line (across triangle base)

    // ==== Initial artifact groups (alliance + row aware) ====
    public static final double RED_ARTIFACT_ROW_START_X = 42.5;    // Red-side starting X for artifact rows (index 0)
    public static final double BLUE_ARTIFACT_ROW_START_X = -53;  // Blue-side starting X for artifact rows (index 0)
    public static final double GPP_ROW_Y = -37.0;                   // Y coordinate for GPP row (toward targets)
    public static final double PGP_ROW_Y = -13.0;                   // Y coordinate for PGP row
    public static final double PPG_ROW_Y = 10.5;                   // Y coordinate for PPG row (furthest toward targets)
    public static final double ARTIFACT_SPACING_X = 5.0;           // Center-to-center spacing along +X for each row, inches
    public static final double ARTIFACT_RADIUS = 2.5;              // Artifact radius used for drawing/spacing (5" diameter)

    // ==== Odometry fusion parameters ====
    public static final double POSE_FILTER_STRENGTH = 0.3;              // Exponential smoothing factor for pose updates (0..1)
    public static final double IMU_HEADING_OFFSET_DEG = 0.0;            // Global heading offset applied to IMU yaw, degrees

    private OdometryConfig() { /* no instances */ }
}

```

## SharedRobotTuning.java

```java
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
 *   - AUTO_DISTANCE_LAST_SEEN_HOLD_MS (ADDED 2026-01-28)
 *       • Auto-only hold window for the last valid vision distance before
 *         AutoRPM returns to its last computed RPM.
 *   - READY_LATCH_TOLERANCE_RPM / READY_LATCH_SETTLE_MS (ADDED 2026-01-05)
 *       • Looser readiness latch used for firing fast-path decisions before a
 *         shot transaction begins.
 *   - FIRING_RECOVERY_RPM_BAND / FIRING_RECOVERY_HOLD_MS / FIRING_RECOVERY_MAX_MS
 *       • Recovery threshold, debounce, and max guard for post-shot RPM rebound.
 *   - FIRING_STREAM_RECOVERY_RPM_BAND / FIRING_STREAM_RECOVERY_MAX_MS
 *       • Streaming-mode recovery band + timeout to keep cadence high between shots.
 *   - SINGLE_BURST_WINDOW_MS / FIRING_BURST_RECOVERY_RPM_BAND / FIRING_BURST_RECOVERY_MAX_MS
 *       • Rapid single-shot burst window and recovery tuning for quick tap sequences.
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
    // CHANGES (2026-01-05): Added readiness latch and recovery-band tunables for firing cadence.
    // CHANGES (2026-01-07): Retuned RPM readiness and added stream/burst recovery tunables.
    // CHANGES (2026-01-28): Added an Auto-only last-seen distance hold window for AutoRPM.
    // --- REV Control Hub IMU physical mounting ---
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;      // Physical face of hub logo; adjust when remounted

    public static RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;    // Direction USB port points; keep consistent with LOGO_DIRECTION

    // --- Launcher speed gate ---
    public static double RPM_TOLERANCE              = 75.0;   // Shared ±RPM window; Launcher.atSpeedToleranceRPM should match
    public static long   RPM_READY_SETTLE_MS        = 80L;    // Time launcher must remain inside tolerance before declaring ready
    public static double READY_LATCH_TOLERANCE_RPM   = 120.0;  // Looser ±RPM window for the continuous readiness latch
    public static long   READY_LATCH_SETTLE_MS       = 80L;    // Time inside latch window before ready latch is set
    public static long   AUTO_DISTANCE_LAST_SEEN_HOLD_MS = 500L; // Auto: hold last valid distance this long when vision drops
    public enum HoldFireForRpmMode {
        ALL,
        INITIAL,
        OFF
    }
    public static HoldFireForRpmMode HOLD_FIRE_FOR_RPM = HoldFireForRpmMode.ALL; // TeleOp feed RPM gate: ALL=every shot, INITIAL=first only, OFF=disabled
    public static double FIRING_RECOVERY_RPM_BAND    = 250.0;  // Recovery band below target where RPM rebound is acceptable
    public static long   FIRING_RECOVERY_HOLD_MS     = 0L;     // Optional debounce inside recovery band before exiting RECOVERING
    public static long   FIRING_RECOVERY_MAX_MS      = 700L;   // Safety timeout to exit RECOVERING even if RPM never rebounds
    public static double FIRING_STREAM_RECOVERY_RPM_BAND = 600.0; // Streaming recovery band below target RPM before resuming cadence
    public static long   FIRING_STREAM_RECOVERY_MAX_MS   = 200L;  // Streaming max recovery time to avoid long cadence stalls
    public static long   SINGLE_BURST_WINDOW_MS          = 350L;  // Tap-to-tap window that marks a single-shot burst
    public static double FIRING_BURST_RECOVERY_RPM_BAND  = 400.0; // Burst recovery band below target RPM for rapid taps
    public static long   FIRING_BURST_RECOVERY_MAX_MS    = 300L;  // Burst max recovery time before forcing the next shot

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

```

## TagAimTuning.java

```java
/*
 * FILE: TagAimTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Collect PD gains, clamp, and deadband used by TagAimController so vision
 *     alignment tweaks stay outside the logic class and mirror the Tunable
 *     Directory.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → AutoAim, targeting, and AprilTag alignment)
 *   - KP / KD
 *       • Proportional and derivative gains converting AprilTag bearing error to
 *         twist power. Increase KP for faster alignment; add KD to calm overshoot.
 *   - CLAMP_ABS
 *       • Absolute value of the twist clamp (±CLAMP_ABS). Keep ≥ AutoAimTuning
 *         MAX_TWIST so AutoAimSpeed can honor its limits without double-clamping.
 *   - DEADBAND_DEG
 *       • Heading window treated as “close enough” to stop jittering around zero.
 */
package org.firstinspires.ftc.teamcode.config;

public final class TagAimTuning {
    private TagAimTuning() {}

    public static double KP = 0.02;           // Proportional gain converting tag bearing error to twist
    public static double KD = 0.003;          // Derivative gain tempering overshoot in twist response
    public static double CLAMP_ABS = 0.6;     // Max absolute twist from TagAimController (±CLAMP_ABS)
    public static double DEADBAND_DEG = 1.5;  // Heading window treated as on-target (deg)
}

```

## TeleOpDriverDefaults.java

```java
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
 *   - FIRING_AUTO_AIM_TIME_THRESHOLD_MS
 *       • Max time spent auto-aiming immediately before a fire request.
 *   - FIRING_SPRAY_DOUBLE_TAP_WINDOW_MS
 *       • Max gap between a tap and a hold to arm spray-mode streaming.
 *   - INITIAL_AUTO_DEFAULT_SPEED
 *       • Local override of SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED for
 *         TeleOp-only experiments.
 */
package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class TeleOpDriverDefaults {
    private TeleOpDriverDefaults() {}

    // CHANGES (2025-10-30): Point intake assist mirror at FeedTuning after tunable move.
    // CHANGES (2025-10-31): Default AutoSpeed + Intake to ON per driver request.
    // CHANGES (2025-11-23): Added AutoRPM tweak scale for on-the-fly AutoSpeed adjustments via D-pad left/right.
    // CHANGES (2025-11-25): Added triple-tap intake reverse gesture window for RB and aligned changelog dates.
    // CHANGES (2026-01-04): Added firing auto-aim window + spray double-tap gesture tunables.
    // CHANGES (2026-01-10): Moved debug telemetry defaults and firing debug flags to DebugTelemetryConfig.
    // Startup toggles
    public static boolean AUTO_SPEED_ENABLED = true;  // TeleOp init default for AutoSpeed toggle
    public static boolean AUTO_AIM_ENABLED   = false; // TeleOp init default for AutoAim toggle
    public static boolean INTAKE_ENABLED     = true;  // TeleOp init default for intake run state

    // Drive brake floor
    public static double SLOWEST_SPEED = 0.25;        // Minimum drive power while brake trigger held

    // Manual RPM slider bounds
    public static double RPM_BOTTOM = 0.0;            // Manual RPM floor when AutoSpeed is off
    public static double RPM_TOP    = 6000.0;         // Manual RPM ceiling when AutoSpeed is off
    public static double AUTORPM_TWEAK_SCALE = 0.02;  // Fractional AutoRPM nudge applied per D-pad press while AutoSpeed is on

    // AutoAim grace + telemetry smoothing
    public static int    AUTO_AIM_LOSS_GRACE_MS = 4000; // Grace window (ms) before AutoAim disables after tag loss
    public static double TELEMETRY_SMOOTH_A     = 0.25;  // Smoothing alpha for range/heading telemetry

    // RPM test adjustments (D-pad)
    public static double RPM_TEST_STEP = 50.0;         // RPM delta per D-pad press while RPM test mode is active

    // Intake assist + auto-speed seed (TeleOp overrides of shared values)
    public static int    INTAKE_ASSIST_MS           = FeedTuning.INTAKE_ASSIST_MS;           // TeleOp copy of post-feed intake assist duration (ms)
    public static int    FIRING_AUTO_AIM_TIME_THRESHOLD_MS = 200; // Max time (ms) to auto-aim just before firing (0 disables)
    public static int    FIRING_SPRAY_DOUBLE_TAP_WINDOW_MS  = 350; // Tap-to-hold window (ms) that arms spray streaming
    public static double INITIAL_AUTO_DEFAULT_SPEED = SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED; // TeleOp seed RPM before first tag lock
    public static int    INTAKE_REVERSE_TAP_WINDOW_MS = 750; // Max window (ms) for detecting three fast RB taps to trigger reverse

    // Auto-stop timer preferences
    public static boolean AUTO_STOP_TIMER_ENABLED = false; // Whether the optional end-of-match timer is active
    public static int     AUTO_STOP_TIMER_TIME_SEC = 119;   // Seconds before AutoStop engages when enabled

}

```

## TeleOpEjectTuning.java

```java
/*
 * FILE: TeleOpEjectTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Keep the TeleOp-only eject routine parameters—temporary launcher RPM and
 *     duration—in one place so clearing jams can be retuned quickly.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Shot cadence, feed, and eject)
 *   - RPM
 *       • Launcher RPM commanded during the eject sequence. Stays clamped by
 *         LauncherTuning.RPM_MIN/RPM_MAX downstream.
 *   - TIME_MS
 *       • Duration of the eject routine before the prior RPM is restored.
 */
package org.firstinspires.ftc.teamcode.config;

public final class TeleOpEjectTuning {
    private TeleOpEjectTuning() {}

    public static double RPM    = 600.0;  // Launcher RPM commanded during eject routine
    public static int    TIME_MS = 1000;  // Duration (ms) to hold eject RPM before restoring prior target
}

```

## TeleOpRumbleTuning.java

```java
/*
 * FILE: TeleOpRumbleTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Centralize the driver-feedback rumble envelopes used in TeleOp so strength,
 *     pulse length, and toggle confirmation cues are easy to adjust between
 *     events without editing TeleOpAllianceBase.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Driver feedback)
 *   - AIM_RUMBLE_ENABLED
 *       • Master enable for manual aim rumble when AutoAim is off.
 *   - AIM_THRESHOLD_DEG / AIM_STRENGTH_MIN/MAX / AIM_PULSE_MIN/MAX / AIM_COOLDOWN_MIN/MAX
 *       • Shape the rumble response window while aiming manually.
 *   - TOGGLE_STRENGTH / TOGGLE_STEP_MS / TOGGLE_GAP_MS
 *       • Haptic pattern used when toggling assists (AutoAim, AutoSpeed, etc.).
 */
package org.firstinspires.ftc.teamcode.config;

public final class TeleOpRumbleTuning {
    private TeleOpRumbleTuning() {}

    // Manual aim rumble defaults
    public static boolean AIM_RUMBLE_ENABLED   = true;  // Master enable for manual aim rumble feedback
    public static double  AIM_THRESHOLD_DEG    = 2.5;   // Heading error (deg) at which rumble begins
    public static double  AIM_STRENGTH_MIN     = 0.10;  // Minimum rumble strength when just outside threshold
    public static double  AIM_STRENGTH_MAX     = 0.65;  // Maximum rumble strength at large heading error
    public static int     AIM_PULSE_MIN_MS     = 120;   // Shortest rumble pulse length (ms) at low error
    public static int     AIM_PULSE_MAX_MS     = 200;   // Longest rumble pulse length (ms) at high error
    public static int     AIM_COOLDOWN_MIN_MS  = 120;   // Minimum cooldown between pulses (ms)
    public static int     AIM_COOLDOWN_MAX_MS  = 350;   // Maximum cooldown between pulses (ms)

    // Toggle confirmation pulses
    public static double TOGGLE_STRENGTH = 0.8; // Strength of each rumble burst for toggle confirmations
    public static int    TOGGLE_STEP_MS  = 120; // Duration (ms) of each rumble step in the toggle pattern
    public static int    TOGGLE_GAP_MS   = 80;  // Gap (ms) between toggle rumble steps
}

```

## VisionConfig.java

```java
package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.vision.VisionAprilTag;

import java.util.Arrays;

/*
 * FILE: VisionConfig.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Centralize Limelight vision selectors and goal tag metadata so TeleOp
 *     and Auto can choose between Limelight and legacy webcam sources without
 *     hard-coding IDs or distances.
 *   - Provide alliance-aware goal tag IDs and field poses for distance
 *     calculations derived from Limelight botpose data.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Vision source selector)
 *   - VISION_SOURCE
 *       • Global switch between the Limelight 3A (default) and the legacy
 *         VisionPortal pipeline.
 *   - LIMELIGHT_RANGE_SCALE
 *       • Multiplier applied to Limelight-derived distances to correct for
 *         mounting height or calibration drift.
 *   - LimelightFusion.*
 *       • Controls pose fusion enablement, quality gating, and XY correction
 *         smoothing when Limelight-based odometry updates are allowed.
 *
 * CHANGES (2025-12-17): Restored a PIPELINE_INDEX alias so TeleOp pipeline
 *                        selection remains compatible after splitting goal vs.
 *                        obelisk pipelines for Limelight AUTO.
 * CHANGES (2025-12-11): Added Limelight pose-fusion tunables (quality, gating,
 *                        axis offsets) to support FTC field-center odometry.
 * CHANGES (2025-12-12): Cleaned imports after odometry package move to keep
 *                        build compatibility with field-center frame updates.
 * CHANGES (2025-12-19): Added Limelight aim-lock tunables to control how long
 *                        goal-tag locks persist and how much hysteresis applies
 *                        before switching aim samples.
 * CHANGES (2025-12-29): Added Limelight localization filter + field-bounds
 *                        tunables to keep pose fusion constrained to goal tags
 *                        and reject off-field botpose updates.
 * CHANGES (2025-12-29): Renamed Limelight localization tunables for clarity and
 *                        added consolidated field-bound limits.
 * CHANGES (2025-12-29): Added Limelight NetworkTables table name for MT2 yaw
 *                        feed and localization filter writes.
 * CHANGES (2025-12-30): Centralized obelisk tag IDs and added a debug toggle
 *                        to reject MT2 fusion frames whenever obelisk tags
 *                        appear in the visible fiducial set.
 * CHANGES (2025-12-30): Added IMU-aligned odometry seeding and adaptive fusion
 *                        tunables so long-distance reacquire corrections remain
 *                        stable while converging quickly.
 * CHANGES (2025-12-31): Refined obelisk handling so localization only rejects
 *                        obelisk-only/primary-only frames, plus added a mixed
 *                        primary guard for obelisk-first solves.
 * CHANGES (2025-12-31): Added a mixed-primary guard that can require multiple
 *                        tags when the Limelight primary ID is obelisk.
 */
public final class VisionConfig {
    private VisionConfig() {}

    public enum VisionSource {
        LIMELIGHT,
        WEBCAM_LEGACY
    }

    public static final VisionSource VISION_SOURCE = VisionSource.LIMELIGHT; // Default to Limelight for heading + distance

    public static final int GOAL_TAG_BLUE = VisionAprilTag.TAG_BLUE_GOAL;    // Blue alliance scoring tag ID
    public static final int GOAL_TAG_RED = VisionAprilTag.TAG_RED_GOAL;      // Red alliance scoring tag ID
    public static final int[] OBELISK_TAG_IDS = {
            VisionAprilTag.TAG_OBELISK_GPP,
            VisionAprilTag.TAG_OBELISK_PGP,
            VisionAprilTag.TAG_OBELISK_PPG
    }; // Obelisk motif tag IDs (21/22/23)

    public static final double GOAL_RED_X_IN = OdometryConfig.TAG_RED_GOAL_X;       // Red goal tag X on field (inches)
    public static final double GOAL_RED_Y_IN = OdometryConfig.TAG_RED_GOAL_Y;       // Red goal tag Y on field (inches)
    public static final double GOAL_BLUE_X_IN = OdometryConfig.TAG_BLUE_GOAL_X;     // Blue goal tag X on field (inches)
    public static final double GOAL_BLUE_Y_IN = OdometryConfig.TAG_BLUE_GOAL_Y;     // Blue goal tag Y on field (inches)

    public static final double LIMELIGHT_RANGE_SCALE = 1.0; // Scaling factor on Limelight distance output; adjust after calibration

    public static final class LimelightAimLock {
        private LimelightAimLock() {}

        public static final long AIM_LOCK_STALE_MS = 250L; // How long to retain a goal lock after loss (ms)
        public static final double AIM_SWITCH_TX_HYST_DEG = 2.0; // Tx delta needed before switching aim sample (deg)
        public static final int AIM_SWITCH_CONFIRM_FRAMES = 2; // Frames required before accepting a new aim sample
    }

    public static final class LimelightFusion {
        private LimelightFusion() {}

        public static final int GOAL_AIM_PIPELINE_INDEX = 0; // LL pipeline used for alliance-goal AprilTag aiming
        public static final int OBELISK_PIPELINE_INDEX = 0; // LL pipeline used for obelisk motif observation
        public static final int PIPELINE_INDEX = GOAL_AIM_PIPELINE_INDEX; // Compatibility alias for TeleOp pipeline selection
        public static final int POLL_HZ = 30; // Limelight polling rate target (Hz)
        public static final boolean ENABLE_POSE_FUSION = false; // Enable LL XY fusion into odometry (when Limelight selected)
        public static final boolean PREFER_MEGA_TAG_2 = true; // Prefer MT2 pose when available
        public static final boolean USE_LLRESULT_BOTPOSE_MT2 = false; // Use LLResult.getBotpose_MT2() when available (diagnostic only)
        public static final String LL_NT_NAME = "limelight"; // NetworkTables name used for MT2 yaw + localization filter writes
        public static final boolean ENABLE_LOCALIZATION_TAG_FILTER = true; // Enable Limelight fiducial whitelist for localization
        public static final int[] LOCALIZATION_VALID_TAG_IDS = {GOAL_TAG_BLUE, GOAL_TAG_RED}; // Allowed tag IDs for localization
        public static final boolean LOCALIZATION_FILTER_APPLY_EVERY_FRAME = true; // Re-send localization filter each loop
        public static final int[] LOCALIZATION_EXCLUDED_TAG_IDS = OBELISK_TAG_IDS; // Tag IDs explicitly excluded from localization
        public static final boolean REQUIRE_2_TAGS_IF_PRIMARY_OBELISK_AND_MIXED = true; // Require >=2 tags when obelisk is primary but goal tag is visible
        public static final boolean DEBUG_REJECT_ON_OBELISK = false; // Debug override: reject MT2 fusion when any obelisk tag is visible
        public static final boolean INIT_ALLOW_VISION_SEED = false; // Allow vision to override explicit Auto start pose during INIT
        public static final boolean ENABLE_LL_LOCALIZATION_TAG_FILTER = true; // Deprecated: use ENABLE_LOCALIZATION_TAG_FILTER
        public static final int[] LL_LOCALIZATION_ALLOWED_TAGS = {GOAL_TAG_BLUE, GOAL_TAG_RED}; // Deprecated: use LOCALIZATION_VALID_TAG_IDS
        public static final double LL_FUSION_FIELD_BOUNDS_IN = 90.0; // Deprecated: superseded by FIELD_HALF_IN - BOUNDS_MARGIN_IN (inches)

        public static final int MIN_VALID_FRAMES = 2; // Require consecutive valid frames before accepting pose
        public static final long MAX_VISION_AGE_MS = 120; // Reject vision results older than this age (ms)
        public static final long MAX_AGE_MS = MAX_VISION_AGE_MS; // Deprecated: use MAX_VISION_AGE_MS
        public static final long YAW_MAX_AGE_MS = 250; // Max age for yaw feed when considering MT2 active (ms)

        public static final double MAX_POS_JUMP_IN_NORMAL = 18.0; // Reject vision if disagreement exceeds this (inches) while tracking
        public static final long REACQUIRE_AFTER_MS = 600; // Enter reacquire mode if no accepted vision for this long (ms)
        public static final double MAX_POS_JUMP_IN_REACQUIRE = 72.0; // Looser reject threshold after tag loss (inches)

        public static final int REACQUIRE_STABLE_FRAMES = 4; // Consecutive stable frames required for confident corrections
        public static final int STABLE_MIN_TAGS = 1; // Minimum tag count for stability qualification
        public static final double STABLE_POSE_DELTA_IN = 6.0; // Max pose delta between frames to count as stable (inches)
        public static final double MAX_POS_STEP_IN_CAUTIOUS = 2.0; // Max translation correction per update in cautious mode (in)
        public static final double MAX_POS_STEP_IN_CONFIDENT = 6.0; // Max translation correction per update in confident mode (in)
        public static final double MAX_HEADING_STEP_DEG_CAUTIOUS = 2.0; // Max heading correction per update in cautious mode (deg)
        public static final double MAX_HEADING_STEP_DEG_CONFIDENT = 6.0; // Max heading correction per update in confident mode (deg)
        public static final double MAX_CORRECTION_STEP_IN = 180.0; // Deprecated: use MAX_POS_STEP_IN_* instead
        public static final double FUSION_ALPHA_NORMAL = 0.35; // Blend factor for clamped corrections during normal tracking
        public static final double FUSION_ALPHA_REACQUIRE = 0.25; // Blend factor for clamped corrections immediately after reacquire

        public static final double MAX_SPEED_IN_PER_S = 35.0; // Skip fusion if robot is faster than this (in/s)
        public static final double MAX_TURN_RATE_DEG_PER_S = 140.0; // Skip fusion if turning faster than this (deg/s)

        public static final double FIELD_HALF_IN = 72.0; // Field half-length (inches) for corner→center transform
        public static final boolean APPLY_CENTER_SHIFT = false; // Apply corner→center shift before offsets
        public static final double BOUNDS_MARGIN_IN = 4.0; // Shrink allowed field bounds by this margin (inches)
        public static final boolean AXIS_SWAP_XY = true; // Swap X/Y axes from Limelight pose (LOCKED)
        public static final int X_SIGN = 1; // Field X sign (LOCKED)
        public static final int Y_SIGN = -1; // Field Y sign (LOCKED)
        public static final double X_OFFSET_IN = 0.0; // Additive X offset if needed (inches)
        public static final double Y_OFFSET_IN = 0.0; // Additive Y offset if needed (inches)
        public static final boolean DEBUG_VERBOSE_VISION = false; // Append extra MT2 frame debug fields to VisionDbg
    }

    public static int goalTagIdForAlliance(Alliance alliance) {
        return alliance == Alliance.RED ? GOAL_TAG_RED : GOAL_TAG_BLUE;
    }

    public static double goalXMeters(Alliance alliance) {
        return inchesToMeters(alliance == Alliance.RED ? GOAL_RED_X_IN : GOAL_BLUE_X_IN);
    }

    public static double goalYMeters(Alliance alliance) {
        return inchesToMeters(alliance == Alliance.RED ? GOAL_RED_Y_IN : GOAL_BLUE_Y_IN);
    }

    public static boolean isObeliskTagId(int tagId) {
        for (int id : OBELISK_TAG_IDS) {
            if (id == tagId) return true;
        }
        return false;
    }

    public static int[] getObeliskTagIds() {
        return Arrays.copyOf(OBELISK_TAG_IDS, OBELISK_TAG_IDS.length);
    }

    private static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }
}

```

## VisionTuning.java

```java
/*
 * FILE: VisionTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Hold vision calibration values—currently the AprilTag range scale—so both
 *     TeleOp and Auto initialize the camera with the same correction factor.
 *   - Define Logitech C270 streaming profiles (resolution, FPS, decimation,
 *     camera controls, calibration) and the default live-view behavior shared
 *     by TeleOp and Autonomous vision helpers.
 *
 * CHANGES (2025-10-31): Consolidated Logitech C270 vision updates.
 *                       Converted VisionTuning to explicit named constants
 *                       per 480p and 720p profile for easy editing, while
 *                       preserving legacy mirror fields and runtime behavior.
 * CHANGES (2025-12-03): Added lighting-normalization tunables (brightness
 *                       target/tolerance, contrast clamps, adaptive
 *                       equalization, and optional INIT exposure nudges) so
 *                       AprilTag processing remains stable across lighting
 *                       conditions without rewriting vision callers.
 * CHANGES (2025-12-09): Added practice vs. event P480 lighting tunables with a
 *                       single environment selector that maps into the
 *                       existing P480 fields so TeleOp/Auto keep using the
 *                       same profile names without changing call sites.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Vision & range calibration)
 *   - RANGE_SCALE
 *       • Multiplier applied to AprilTag ftcPose.range (meters) to correct for
 *         camera height or lens characteristics. Compute as
 *         true_distance_m / measured_distance_m during calibration.
 */
package org.firstinspires.ftc.teamcode.config;

public final class VisionTuning {
    private VisionTuning() {}

    // === Logitech C270 AprilTag tuning (edit values below) ===

    public enum Environment {
        PRACTICE,
        EVENT
    }

    // Lighting environment selector (PRACTICE default to preserve current behavior)
    public static Environment VISION_ENVIRONMENT = Environment.PRACTICE;  // Choose PRACTICE for our arena, EVENT for bright fields

    // Practice-field P480 lighting (current baseline)
    public static final int P480_PRACTICE_EXPOSURE_MS = 6;                // Practice exposure tuned for our arena lighting
    public static final int P480_PRACTICE_GAIN = 85;                      // Practice gain tuned for our arena lighting
    public static final boolean P480_PRACTICE_WHITE_BALANCE_LOCK = true;  // Practice white balance lock for stable color

    // Event-field P480 lighting (bright venue baseline)
    public static final int P480_EVENT_EXPOSURE_MS = 2;                   // Event exposure tuned for bright fields
    public static final int P480_EVENT_GAIN = 50;                         // Event gain tuned for bright fields
    public static final boolean P480_EVENT_WHITE_BALANCE_LOCK = true;     // Event white balance lock for stable color

    // Startup defaults for the vision system
    public static final Mode DEFAULT_MODE = Mode.P480;                 // Startup profile (Performance 640×480 for Control Hub headroom)
    public static final boolean DEFAULT_LIVE_VIEW_ENABLED = false;     // Disable live preview by default; toggle via Gamepad 2 D-pad up/down

    // AprilTag distance calibration multiplier (unitless)
    public static double RANGE_SCALE = 0.03;                           // Scale ftcPose.range so physical distance matches tape-measured truth

    // Logitech C270 "Performance" profile (640×480 @ 30 FPS)
    public static final String P480_NAME = "P480";                     // Short label shown in telemetry when Performance mode is active
    public static final int P480_WIDTH = 640;                           // Camera capture width in pixels for Performance profile
    public static final int P480_HEIGHT = 480;                          // Camera capture height in pixels for Performance profile
    public static final int P480_FPS = 30;                              // Target frame rate for Performance profile (Control Hub friendly)
    public static final float P480_DECIMATION = 2.0f;                   // AprilTag decimation (higher skips pixels for speed)
    public static final int P480_PROCESS_EVERY_N = 1;                   // Process every frame (no skipping) when in Performance mode
    public static final double P480_MIN_DECISION_MARGIN = 10.0;         // Reject detections with weaker decision margins than this threshold
    public static int P480_EXPOSURE_MS = P480_PRACTICE_EXPOSURE_MS;     // Manual exposure in milliseconds (environment-mapped)
    public static int P480_GAIN = P480_PRACTICE_GAIN;                   // Camera-native gain for stable image brightness at 480p
    public static boolean P480_WHITE_BALANCE_LOCK = P480_PRACTICE_WHITE_BALANCE_LOCK; // Lock white balance after start to prevent drift
    public static final double P480_FX = 690.0;                         // Calibrated focal length (pixels) in X for 480p profile
    public static final double P480_FY = 690.0;                         // Calibrated focal length (pixels) in Y for 480p profile
    public static final double P480_CX = 320.0;                         // Principal point X (pixels) for 480p profile
    public static final double P480_CY = 240.0;                         // Principal point Y (pixels) for 480p profile
    public static final double P480_K1 = -0.27;                         // Brown–Conrady radial distortion k1 for 480p profile
    public static final double P480_K2 = 0.09;                          // Brown–Conrady radial distortion k2 for 480p profile
    public static final double P480_P1 = 0.0008;                        // Brown–Conrady tangential distortion p1 for 480p profile
    public static final double P480_P2 = -0.0006;                       // Brown–Conrady tangential distortion p2 for 480p profile
    public static final double P480_K3 = 0.0;                           // Brown–Conrady radial distortion k3 for 480p profile

    // Logitech C270 "Sighting" profile (1280×720 @ 20 FPS)
    public static final String P720_NAME = "P720";                     // Short label shown in telemetry when Sighting mode is active
    public static final int P720_WIDTH = 1280;                          // Camera capture width in pixels for Sighting profile
    public static final int P720_HEIGHT = 720;                          // Camera capture height in pixels for Sighting profile
    public static final int P720_FPS = 20;                              // Target frame rate for Sighting profile (720p streaming)
    public static final float P720_DECIMATION = 2.2f;                   // AprilTag decimation tuned for longer range detail
    public static final int P720_PROCESS_EVERY_N = 2;                   // Process every other frame for 720p to balance CPU load
    public static final double P720_MIN_DECISION_MARGIN = 24.0;         // Minimum AprilTag decision margin accepted at 720p
    public static final int P720_EXPOSURE_MS = 7;                      // Manual exposure (ms) for 720p long-range lighting
    public static final int P720_GAIN = 85;                            // Camera-native gain for 720p profile brightness
    public static final boolean P720_WHITE_BALANCE_LOCK = true;         // Lock white balance for consistent color at 720p
    public static final double P720_FX = 1380.0;                        // Calibrated focal length (pixels) in X for 720p profile
    public static final double P720_FY = 1035.0;                        // Calibrated focal length (pixels) in Y for 720p profile
    public static final double P720_CX = 640.0;                         // Principal point X (pixels) for 720p profile
    public static final double P720_CY = 360.0;                         // Principal point Y (pixels) for 720p profile
    public static final double P720_K1 = -0.23;                         // Brown–Conrady radial distortion k1 for 720p profile
    public static final double P720_K2 = 0.06;                          // Brown–Conrady radial distortion k2 for 720p profile
    public static final double P720_P1 = 0.0005;                        // Brown–Conrady tangential distortion p1 for 720p profile
    public static final double P720_P2 = -0.0005;                       // Brown–Conrady tangential distortion p2 for 720p profile
    public static final double P720_K3 = 0.0;                           // Brown–Conrady radial distortion k3 for 720p profile

    // === Do not edit below: helper structures & derived mirrors ===

    public enum Mode {
        P480,
        P720
    }

    public static final class Profile {
        public final String name;
        public final int width;
        public final int height;
        public final int fps;
        public final float decimation;
        public final int processEveryN;
        public final double minDecisionMargin;
        public final int exposureMs;
        public final int gain;
        public final boolean whiteBalanceLock;
        public final double fx;
        public final double fy;
        public final double cx;
        public final double cy;
        public final double k1;
        public final double k2;
        public final double p1;
        public final double p2;
        public final double k3;

        public Profile(String name,
                       int width,
                       int height,
                       int fps,
                       float decimation,
                       int processEveryN,
                       double minDecisionMargin,
                       int exposureMs,
                       int gain,
                       boolean whiteBalanceLock,
                       double fx,
                       double fy,
                       double cx,
                       double cy,
                       double k1,
                       double k2,
                       double p1,
                       double p2,
                       double k3) {
            this.name = name;
            this.width = width;
            this.height = height;
            this.fps = fps;
            this.decimation = decimation;
            this.processEveryN = Math.max(1, processEveryN);
            this.minDecisionMargin = Math.max(0.0, minDecisionMargin);
            this.exposureMs = Math.max(0, exposureMs);
            this.gain = Math.max(0, gain);
            this.whiteBalanceLock = whiteBalanceLock;
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
            this.k1 = k1;
            this.k2 = k2;
            this.p1 = p1;
            this.p2 = p2;
            this.k3 = k3;
        }

        public boolean hasIntrinsics() {
            return isFinitePositive(fx) && isFinitePositive(fy)
                    && isFinite(cx) && isFinite(cy);
        }

        public boolean hasDistortion() {
            return Math.abs(k1) > 1e-9
                    || Math.abs(k2) > 1e-9
                    || Math.abs(p1) > 1e-9
                    || Math.abs(p2) > 1e-9
                    || Math.abs(k3) > 1e-9;
        }

        private boolean isFinitePositive(double value) {
            return !Double.isNaN(value) && !Double.isInfinite(value) && value > 0.0;
        }

        private boolean isFinite(double value) {
            return !Double.isNaN(value) && !Double.isInfinite(value);
        }
    }

    public static Profile forMode(Mode mode) {
        Mode resolved = (mode != null) ? mode : Mode.P480;
        switch (resolved) {
            case P720:
                return new Profile(
                        P720_NAME,
                        P720_WIDTH,
                        P720_HEIGHT,
                        P720_FPS,
                        P720_DECIMATION,
                        P720_PROCESS_EVERY_N,
                        P720_MIN_DECISION_MARGIN,
                        P720_EXPOSURE_MS,
                        P720_GAIN,
                        P720_WHITE_BALANCE_LOCK,
                        P720_FX,
                        P720_FY,
                        P720_CX,
                        P720_CY,
                        P720_K1,
                        P720_K2,
                        P720_P1,
                        P720_P2,
                        P720_K3
                );
            case P480:
            default:
                return new Profile(
                        P480_NAME,
                        P480_WIDTH,
                        P480_HEIGHT,
                        P480_FPS,
                        P480_DECIMATION,
                        P480_PROCESS_EVERY_N,
                        P480_MIN_DECISION_MARGIN,
                        P480_EXPOSURE_MS,
                        P480_GAIN,
                        P480_WHITE_BALANCE_LOCK,
                        P480_FX,
                        P480_FY,
                        P480_CX,
                        P480_CY,
                        P480_K1,
                        P480_K2,
                        P480_P1,
                        P480_P2,
                        P480_K3
                );
        }
    }

    public static Profile PROFILE_480 = forMode(Mode.P480);
    public static Profile PROFILE_720 = forMode(Mode.P720);

    public static Profile DEFAULT_PROFILE = forMode(DEFAULT_MODE);

    // Legacy single-profile fields mirror the current default profile so
    // existing references continue to compile while telemetry migrates.
    public static int VISION_RES_WIDTH = DEFAULT_PROFILE.width;      // px
    public static int VISION_RES_HEIGHT = DEFAULT_PROFILE.height;    // px
    public static int VISION_TARGET_FPS = DEFAULT_PROFILE.fps;       // frames per second
    public static float APRILTAG_DECIMATION = DEFAULT_PROFILE.decimation; // unitless decimation
    public static int VISION_PROCESS_EVERY_N = DEFAULT_PROFILE.processEveryN;
    public static double MIN_DECISION_MARGIN = DEFAULT_PROFILE.minDecisionMargin;
    public static int EXPOSURE_MS = DEFAULT_PROFILE.exposureMs;      // manual exposure in milliseconds
    public static int GAIN = DEFAULT_PROFILE.gain;                   // camera-native gain units
    public static boolean WHITE_BALANCE_LOCK_ENABLED = DEFAULT_PROFILE.whiteBalanceLock;

    static {
        applyEnvironment();
    }

    public static void applyEnvironment() {
        Environment selected = (VISION_ENVIRONMENT != null) ? VISION_ENVIRONMENT : Environment.PRACTICE;
        switch (selected) {
            case EVENT:
                P480_EXPOSURE_MS = clampExposure(P480_EVENT_EXPOSURE_MS);
                P480_GAIN = clampGain(P480_EVENT_GAIN);
                P480_WHITE_BALANCE_LOCK = P480_EVENT_WHITE_BALANCE_LOCK;
                break;
            case PRACTICE:
            default:
                P480_EXPOSURE_MS = clampExposure(P480_PRACTICE_EXPOSURE_MS);
                P480_GAIN = clampGain(P480_PRACTICE_GAIN);
                P480_WHITE_BALANCE_LOCK = P480_PRACTICE_WHITE_BALANCE_LOCK;
                break;
        }

        refreshDerivedProfiles();
    }

    private static int clampExposure(int exposureMs) {
        return Math.max(0, exposureMs);
    }

    private static int clampGain(int gain) {
        return Math.max(0, gain);
    }

    public static void refreshDerivedProfiles() {
        PROFILE_480 = forMode(Mode.P480);
        PROFILE_720 = forMode(Mode.P720);
        DEFAULT_PROFILE = forMode(DEFAULT_MODE);

        VISION_RES_WIDTH = DEFAULT_PROFILE.width;
        VISION_RES_HEIGHT = DEFAULT_PROFILE.height;
        VISION_TARGET_FPS = DEFAULT_PROFILE.fps;
        APRILTAG_DECIMATION = DEFAULT_PROFILE.decimation;
        VISION_PROCESS_EVERY_N = DEFAULT_PROFILE.processEveryN;
        MIN_DECISION_MARGIN = DEFAULT_PROFILE.minDecisionMargin;
        EXPOSURE_MS = DEFAULT_PROFILE.exposureMs;
        GAIN = DEFAULT_PROFILE.gain;
        WHITE_BALANCE_LOCK_ENABLED = DEFAULT_PROFILE.whiteBalanceLock;

        HAS_480P_INTRINSICS = PROFILE_480.hasIntrinsics();
        FX_480 = PROFILE_480.fx;
        FY_480 = PROFILE_480.fy;
        CX_480 = PROFILE_480.cx;
        CY_480 = PROFILE_480.cy;

        HAS_480P_DISTORTION = PROFILE_480.hasDistortion();
        K1_480 = PROFILE_480.k1;
        K2_480 = PROFILE_480.k2;
        P1_480 = PROFILE_480.p1;
        P2_480 = PROFILE_480.p2;
        K3_480 = PROFILE_480.k3;

        HAS_720P_INTRINSICS = PROFILE_720.hasIntrinsics();
        FX_720 = PROFILE_720.fx;
        FY_720 = PROFILE_720.fy;
        CX_720 = PROFILE_720.cx;
        CY_720 = PROFILE_720.cy;

        HAS_720P_DISTORTION = PROFILE_720.hasDistortion();
        K1_720 = PROFILE_720.k1;
        K2_720 = PROFILE_720.k2;
        P1_720 = PROFILE_720.p1;
        P2_720 = PROFILE_720.p2;
        K3_720 = PROFILE_720.k3;
    }

    // Lighting normalization toggles and parameters (AprilTag pre-processing)
    public static boolean ENABLE_BRIGHTNESS_NORMALIZATION = true;    // Global alpha/beta frame normalization toggle
    public static boolean ENABLE_ADAPTIVE_EQUALIZATION = true;      // Enable CLAHE-style adaptive equalization after alpha/beta
    public static double TARGET_MEAN_BRIGHTNESS = 120.0;             // Target grayscale mean (0–255) to aim toward before tag solve
    public static double BRIGHTNESS_TOLERANCE = 8.0;                 // Allowed delta from target mean before any adjustment occurs
    public static double MIN_CONTRAST_GAIN = 0.9;                    // Lower clamp for contrast gain (alpha)
    public static double MAX_CONTRAST_GAIN = 1.2;                    // Upper clamp for contrast gain (alpha)
    public static double MAX_BRIGHTNESS_OFFSET = 25.0;               // Maximum |beta| brightness shift per frame (pixel units)
    public static int BRIGHTNESS_SMOOTHING_WINDOW = 5;               // Frames in the moving-average window for mean brightness
    public static double MAX_PER_FRAME_ADJUST_DELTA = 0.08;          // Max change per frame for alpha/beta to prevent flicker

    // Optional adaptive equalization details (applied after alpha/beta when enabled)
    public static double ADAPTIVE_CLIP_LIMIT = 2.0;                  // CLAHE clip limit to bound contrast expansion
    public static int ADAPTIVE_TILE_GRID_SIZE = 8;                   // Square tile grid size (pixels) used by adaptive equalizer

    // Optional INIT-time exposure nudging (single bounded correction before START)
    public static boolean ENABLE_INIT_EXPOSURE_TUNING = true;        // Toggle a one-time exposure nudge during INIT based on brightness
    public static double INIT_EXPOSURE_TARGET_MEAN = 120.0;          // Target mean used for INIT exposure tuning (matches TARGET_MEAN_BRIGHTNESS by default)
    public static double INIT_EXPOSURE_TOLERANCE = 10.0;             // Allowed band around INIT target before nudging exposure
    public static int INIT_EXPOSURE_MAX_STEPS = 2;                   // Max number of single-step exposure nudges applied during INIT

    // Goal-tag visibility smoothing + aim margin flex (all units: frames or margin points)
    public static final int GOAL_VISIBILITY_ON_STREAK = 3;            // Frames required to declare smoothed goal visibility ON
    public static final int GOAL_VISIBILITY_OFF_STREAK = 5;           // Frames required to declare smoothed goal visibility OFF
    public static final double GOAL_TAG_MIN_MARGIN_FLOOR = 8.0;       // Lowest allowable decision margin when flexing thresholds
    public static final double GOAL_TAG_P480_MARGIN_FLEX = 2.0;       // Amount to relax the decision margin in P480 for aim gating

    // Vision health sampling thresholds
    public static final double HEALTH_PASS_GOOD_RATIO = 0.80;         // Minimum good/total ratio for a PASS classification
    public static final double HEALTH_WARN_GOOD_RATIO = 0.40;         // Minimum good/total ratio for a WARN classification
    public static final double HEALTH_MARGIN_WARN_DELTA = 2.0;        // Allowed shortfall (points) below profile minDecisionMargin before FAIL
    public static final double HEALTH_BRIGHTNESS_WARN_DELTA = 15.0;   // Allowed deviation (points) from target mean before lighting is considered extreme

    // 640x480 calibrated intrinsics (Logitech C270 default profile)
    public static boolean HAS_480P_INTRINSICS = PROFILE_480.hasIntrinsics();
    public static double FX_480 = PROFILE_480.fx;
    public static double FY_480 = PROFILE_480.fy;
    public static double CX_480 = PROFILE_480.cx;
    public static double CY_480 = PROFILE_480.cy;

    // 640x480 Brown-Conrady distortion coefficients (k1, k2, p1, p2, k3)
    public static boolean HAS_480P_DISTORTION = PROFILE_480.hasDistortion();
    public static double K1_480 = PROFILE_480.k1;
    public static double K2_480 = PROFILE_480.k2;
    public static double P1_480 = PROFILE_480.p1;
    public static double P2_480 = PROFILE_480.p2;
    public static double K3_480 = PROFILE_480.k3;

    // 1280x720 calibrated intrinsics (Logitech C270 default profile)
    public static boolean HAS_720P_INTRINSICS = PROFILE_720.hasIntrinsics();
    public static double FX_720 = PROFILE_720.fx;
    public static double FY_720 = PROFILE_720.fy;
    public static double CX_720 = PROFILE_720.cx;
    public static double CY_720 = PROFILE_720.cy;

    // 1280x720 Brown-Conrady distortion coefficients (k1, k2, p1, p2, k3)
    public static boolean HAS_720P_DISTORTION = PROFILE_720.hasDistortion();
    public static double K1_720 = PROFILE_720.k1;
    public static double K2_720 = PROFILE_720.k2;
    public static double P1_720 = PROFILE_720.p1;
    public static double P2_720 = PROFILE_720.p2;
    public static double K3_720 = PROFILE_720.k3;
}

```
