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
        public static final boolean ENABLE_POSE_FUSION = true; // Enable LL XY fusion into odometry (when Limelight selected)
        public static final boolean PREFER_MEGA_TAG_2 = true; // Prefer MT2 pose when available
        public static final boolean USE_LLRESULT_BOTPOSE_MT2 = false; // Use LLResult.getBotpose_MT2() when available (diagnostic only)
        public static final String LL_NT_NAME = "limelight"; // NetworkTables name used for MT2 yaw + localization filter writes
        public static final boolean ENABLE_LOCALIZATION_TAG_FILTER = true; // Enable Limelight fiducial whitelist for localization
        public static final int[] LOCALIZATION_VALID_TAG_IDS = {GOAL_TAG_BLUE, GOAL_TAG_RED}; // Allowed tag IDs for localization
        public static final boolean LOCALIZATION_FILTER_APPLY_EVERY_FRAME = true; // Re-send localization filter each loop
        public static final int[] LOCALIZATION_EXCLUDED_TAG_IDS = OBELISK_TAG_IDS; // Tag IDs explicitly excluded from localization
        public static final boolean DEBUG_REJECT_ON_OBELISK = true; // Reject MT2 fusion frames when any obelisk tag is visible
        public static final boolean ENABLE_LL_LOCALIZATION_TAG_FILTER = true; // Deprecated: use ENABLE_LOCALIZATION_TAG_FILTER
        public static final int[] LL_LOCALIZATION_ALLOWED_TAGS = {GOAL_TAG_BLUE, GOAL_TAG_RED}; // Deprecated: use LOCALIZATION_VALID_TAG_IDS
        public static final double LL_FUSION_FIELD_BOUNDS_IN = 90.0; // Deprecated: superseded by FIELD_HALF_IN - BOUNDS_MARGIN_IN (inches)

        public static final int MIN_VALID_FRAMES = 2; // Require consecutive valid frames before accepting pose
        public static final long MAX_AGE_MS = 120; // Reject vision results older than this age (ms)
        public static final long YAW_MAX_AGE_MS = 250; // Max age for yaw feed when considering MT2 active (ms)

        public static final double MAX_POS_JUMP_IN_NORMAL = 18.0; // Reject vision if disagreement exceeds this (inches) while tracking
        public static final long REACQUIRE_AFTER_MS = 600; // Enter reacquire mode if no accepted vision for this long (ms)
        public static final double MAX_POS_JUMP_IN_REACQUIRE = 72.0; // Looser reject threshold after tag loss (inches)

        public static final double MAX_CORRECTION_STEP_IN = 180.0; // Clamp per-update correction magnitude (inches)
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
