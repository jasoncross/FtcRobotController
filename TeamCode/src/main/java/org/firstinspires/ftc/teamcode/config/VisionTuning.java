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
    public static final int P480_EXPOSURE_MS = 2;                      // Manual exposure in milliseconds tuned for indoor lighting
    public static final int P480_GAIN = 50;                             // Camera-native gain for stable image brightness at 480p
    public static final boolean P480_WHITE_BALANCE_LOCK = true;         // Lock white balance after start to prevent drift
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

    public static final Profile PROFILE_480 = forMode(Mode.P480);
    public static final Profile PROFILE_720 = forMode(Mode.P720);

    public static final Profile DEFAULT_PROFILE = PROFILE_480;

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
