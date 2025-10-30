/*
 * FILE: VisionTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Hold vision calibration values—currently the AprilTag range scale—so both
 *     TeleOp and Auto initialize the camera with the same correction factor.
 *   - Surface camera operating targets (resolution / FPS / exposure / gain)
 *     that TeleOp applies immediately after the VisionPortal starts streaming.
 *
 * CHANGES (2025-11-03): Added Logitech C270 camera tunables for 1280x720@20 FPS
 *                       with manual exposure/gain + white balance lock.
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

    // AprilTag distance calibration multiplier (unitless)
    public static double RANGE_SCALE = 0.03;

    // Logitech C270 operating targets for long-range AprilTags
    public static int VISION_RES_WIDTH = 1280;      // px
    public static int VISION_RES_HEIGHT = 720;      // px
    public static int VISION_TARGET_FPS = 20;       // frames per second
    public static float APRILTAG_DECIMATION = 2.0f; // unitless decimation inside AprilTagProcessor
    public static int EXPOSURE_MS = 15;             // manual exposure in milliseconds
    public static int GAIN = 110;                   // camera-native gain units
    public static boolean WHITE_BALANCE_LOCK_ENABLED = true; // lock WB once stream is ready
}
