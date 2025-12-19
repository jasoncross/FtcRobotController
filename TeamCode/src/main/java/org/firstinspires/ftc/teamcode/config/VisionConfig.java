package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.vision.VisionAprilTag;

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

    public static final double GOAL_RED_X_IN = OdometryConfig.TAG_RED_GOAL_X;       // Red goal tag X on field (inches)
    public static final double GOAL_RED_Y_IN = OdometryConfig.TAG_RED_GOAL_Y;       // Red goal tag Y on field (inches)
    public static final double GOAL_BLUE_X_IN = OdometryConfig.TAG_BLUE_GOAL_X;     // Blue goal tag X on field (inches)
    public static final double GOAL_BLUE_Y_IN = OdometryConfig.TAG_BLUE_GOAL_Y;     // Blue goal tag Y on field (inches)

    public static final double LIMELIGHT_RANGE_SCALE = 1.0; // Scaling factor on Limelight distance output; adjust after calibration

    public static final class AimLock {
        private AimLock() {}

        public static final long AIM_LOCK_STALE_MS = 250L; // How long to keep a goal aim lock after the last sighting before clearing
        public static final double AIM_SWITCH_TX_HYST_DEG = 2.0; // Required tx improvement (deg) before accepting a new locked sample
        public static final int AIM_SWITCH_CONFIRM_FRAMES = 2; // Consecutive frames needed to confirm a new locked tx sample
    }

    public static final class LimelightFusion {
        private LimelightFusion() {}

        public static final int GOAL_AIM_PIPELINE_INDEX = 0; // LL pipeline used for alliance-goal AprilTag aiming
        public static final int OBELISK_PIPELINE_INDEX = 0; // LL pipeline used for obelisk motif observation
        public static final int PIPELINE_INDEX = GOAL_AIM_PIPELINE_INDEX; // Compatibility alias for TeleOp pipeline selection
        public static final int POLL_HZ = 30; // Limelight polling rate target (Hz)
        public static final boolean ENABLE_POSE_FUSION = true; // Enable LL XY fusion into odometry (when Limelight selected)
        public static final boolean PREFER_MEGA_TAG_2 = true; // Prefer MT2 pose when available

        public static final int MIN_VALID_FRAMES = 2; // Require consecutive valid frames before accepting pose
        public static final long MAX_AGE_MS = 120; // Reject vision results older than this age (ms)

        public static final double MAX_POS_JUMP_IN_NORMAL = 18.0; // Reject vision if disagreement exceeds this (inches) while tracking
        public static final long REACQUIRE_AFTER_MS = 600; // Enter reacquire mode if no accepted vision for this long (ms)
        public static final double MAX_POS_JUMP_IN_REACQUIRE = 72.0; // Looser reject threshold after tag loss (inches)

        public static final double MAX_CORRECTION_STEP_IN = 3.0; // Clamp per-update correction magnitude (inches)
        public static final double FUSION_ALPHA_NORMAL = 0.35; // Blend factor for clamped corrections during normal tracking
        public static final double FUSION_ALPHA_REACQUIRE = 0.25; // Blend factor for clamped corrections immediately after reacquire

        public static final double MAX_SPEED_IN_PER_S = 35.0; // Skip fusion if robot is faster than this (in/s)
        public static final double MAX_TURN_RATE_DEG_PER_S = 140.0; // Skip fusion if turning faster than this (deg/s)

        public static final boolean AXIS_SWAP_XY = false; // Swap X/Y axes from Limelight pose if needed
        public static final int X_SIGN = 1; // Flip X axis if needed (+1 normal)
        public static final int Y_SIGN = 1; // Flip Y axis if needed (+1 normal)
        public static final double X_OFFSET_IN = 0.0; // Additive X offset if needed (inches)
        public static final double Y_OFFSET_IN = 0.0; // Additive Y offset if needed (inches)
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

    private static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }
}
