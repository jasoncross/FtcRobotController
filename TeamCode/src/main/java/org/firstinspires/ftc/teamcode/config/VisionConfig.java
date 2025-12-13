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
 *   - CameraFusion.*
 *       • Controls pose fusion enablement, quality gating, and XY correction
 *         smoothing when camera-based odometry updates are allowed.
 *
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

    public static final class CameraFusion {
        private CameraFusion() {}

        public static final int PIPELINE_INDEX = 0; // Default LL pipeline index for AprilTags/localization
        public static final int POLL_HZ = 30; // Limelight polling rate target (Hz)
        public static final boolean ENABLE_POSE_FUSION = true; // Enable camera XY fusion into odometry when selected
        public static final boolean PREFER_MEGA_TAG_2 = true; // Prefer MT2 pose when available

        public static final int CAMERA_POSE_WINDOW_FRAMES = 10; // Window size for stability gate (frames)
        public static final int CAMERA_POSE_MIN_GOOD_FRAMES = 4; // Minimum frames required for a stable lock
        public static final double CAMERA_POSE_MAX_STDDEV_IN = 6.0; // Max XY stddev inside the stability window (in)
        public static final double CAMERA_POSE_MAX_STDDEV_DEG = 5.0; // Max heading stddev inside the stability window (deg)
        public static final long CAMERA_POSE_MAX_AGE_MS = 200; // Max age of samples considered for stability (ms)

        public static final double MAX_CORRECTION_STEP_IN = 3.0; // Clamp per-update correction magnitude (inches)
        public static final double FUSION_ALPHA = 0.15; // Blend factor for clamped corrections while stable

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
