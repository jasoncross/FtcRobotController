package org.firstinspires.ftc.teamcode.config;

/*
 * FILE: OdometryConfig.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Centralize all odometry + field layout tunables so the fused pose, camera
 *     offsets, and Dashboard drawings stay aligned across TeleOp and Auto.
 *   - Coordinates follow the DECODE field convention: (0,0) at the center of
 *     the human-player wall, +X to the right when facing the targets, +Y toward
 *     the targets.
 *
 * NOTES
 *   - Update these values with real measurements from the competition robot and
 *     practice field; every constant includes units and an inline description
 *     for quick pit-side edits.
 *
 * CHANGES (2025-11-25): Added goal AprilTag pose tunables (XYZ + yaw) to support
 *                        tag-based odometry fusion with camera offsets.
 */
public final class OdometryConfig {

    // ==== Coordinate system & robot geometry (inches) ====
    public static final double FIELD_LENGTH = 144.0;        // inches, Y span from human wall to target wall
    public static final double FIELD_WIDTH  = 144.0;        // inches, X span from left to right wall
    public static final double HUMAN_WALL_Y = 0.0;          // Y position of the human wall
    public static final double TARGET_WALL_Y = 144.0;       // Y position of the target wall
    public static final double FIELD_CENTER_X = 0.0;        // X origin at field centerline
    public static final double LEFT_FIELD_X  = -72.0;       // Left field edge (human perspective)
    public static final double RIGHT_FIELD_X = 72.0;        // Right field edge

    public static final double ROBOT_HALF_LENGTH = 9.0;     // Half-length of robot footprint, inches
    public static final double ROBOT_HALF_WIDTH  = 9.0;     // Half-width of robot footprint, inches

    public static final double INTAKE_OFFSET_X = 10.0;      // Intake center offset forward (+X) from robot center, inches
    public static final double INTAKE_OFFSET_Y = 0.0;       // Intake center offset left (+Y) from robot center, inches

    public static final double LAUNCHER_OFFSET_X = -4.0;    // Launcher exit offset forward (+X) from robot center, inches
    public static final double LAUNCHER_OFFSET_Y = 0.0;     // Launcher exit offset left (+Y) from robot center, inches

    public static final double CAMERA_OFFSET_X = 6.0;       // Camera offset forward (+X) from robot center, inches
    public static final double CAMERA_OFFSET_Y = 0.0;       // Camera offset left (+Y) from robot center, inches
    public static final double CAMERA_OFFSET_Z = 10.0;      // Camera height above floor, inches
    public static final double CAMERA_PITCH_DEG = -10.0;    // Camera pitch downward (-) relative to robot frame, degrees
    public static final double CAMERA_YAW_DEG   = 0.0;      // Camera yaw relative to robot heading, degrees

    // ==== AprilTag goal poses (inches/degrees) ====
    public static final double TAG_RED_GOAL_X       = -60.0;   // Red goal AprilTag center X, inches
    public static final double TAG_RED_GOAL_Y       = 132.0;   // Red goal AprilTag center Y, inches
    public static final double TAG_RED_GOAL_Z       = 29.5;    // Red goal AprilTag center height above floor, inches
    public static final double TAG_RED_GOAL_YAW_DEG = 135.0;   // Red goal AprilTag yaw facing field, degrees

    public static final double TAG_BLUE_GOAL_X       = 60.0;   // Blue goal AprilTag center X, inches
    public static final double TAG_BLUE_GOAL_Y       = 132.0;  // Blue goal AprilTag center Y, inches
    public static final double TAG_BLUE_GOAL_Z       = 29.5;   // Blue goal AprilTag center height above floor, inches
    public static final double TAG_BLUE_GOAL_YAW_DEG = 225.0;  // Blue goal AprilTag yaw facing field, degrees

    // ==== Field elements – goals & classifiers (inches) ====
    public static final double GOAL_RED_X  = -60.0;         // Red goal tag center X
    public static final double GOAL_RED_Y  = 144.0;         // Red goal tag center Y
    public static final double GOAL_BLUE_X = 60.0;          // Blue goal tag center X
    public static final double GOAL_BLUE_Y = 144.0;         // Blue goal tag center Y

    public static final double CLASSIFIER_RED_X  = -36.0;   // Red classifier center X
    public static final double CLASSIFIER_RED_Y  = 132.0;   // Red classifier center Y
    public static final double CLASSIFIER_BLUE_X = 36.0;    // Blue classifier center X
    public static final double CLASSIFIER_BLUE_Y = 132.0;   // Blue classifier center Y

    // ==== Gates, gate zones, secret tunnel (inches) ====
    public static final double GATE_RED_X  = -60.0;         // Red gate bar center X
    public static final double GATE_RED_Y  = 132.0;         // Red gate bar center Y
    public static final double GATE_BLUE_X = 60.0;          // Blue gate bar center X
    public static final double GATE_BLUE_Y = 132.0;         // Blue gate bar center Y

    public static final double GATE_ZONE_RED_CENTER_X  = -48.0;  // Red gate zone center X
    public static final double GATE_ZONE_RED_CENTER_Y  = 108.0;  // Red gate zone center Y
    public static final double GATE_ZONE_BLUE_CENTER_X = 48.0;   // Blue gate zone center X
    public static final double GATE_ZONE_BLUE_CENTER_Y = 108.0;  // Blue gate zone center Y
    public static final double GATE_ZONE_WIDTH = 24.0;           // Gate zone width, inches
    public static final double GATE_ZONE_DEPTH = 12.0;           // Gate zone depth, inches

    public static final double SECRET_TUNNEL_RED_X  = -72.0;     // Red wall secret tunnel X
    public static final double SECRET_TUNNEL_RED_Y1 = 24.0;      // Red tunnel start Y
    public static final double SECRET_TUNNEL_RED_Y2 = 96.0;      // Red tunnel end Y
    public static final double SECRET_TUNNEL_BLUE_X  = 72.0;     // Blue wall secret tunnel X
    public static final double SECRET_TUNNEL_BLUE_Y1 = 24.0;     // Blue tunnel start Y
    public static final double SECRET_TUNNEL_BLUE_Y2 = 96.0;     // Blue tunnel end Y

    // ==== Loading zones & base zones (inches) ====
    public static final double LOADING_ZONE_RED_CENTER_X  = -60.0; // Red loading zone center X
    public static final double LOADING_ZONE_RED_CENTER_Y  = 24.0;  // Red loading zone center Y
    public static final double LOADING_ZONE_BLUE_CENTER_X = 60.0;  // Blue loading zone center X
    public static final double LOADING_ZONE_BLUE_CENTER_Y = 24.0;  // Blue loading zone center Y
    public static final double LOADING_ZONE_SIZE = 24.0;           // Loading zone square size, inches

    public static final double BASE_ZONE_RED_CENTER_X  = -24.0;    // Red base zone center X
    public static final double BASE_ZONE_RED_CENTER_Y  = 48.0;     // Red base zone center Y
    public static final double BASE_ZONE_BLUE_CENTER_X = 24.0;     // Blue base zone center X
    public static final double BASE_ZONE_BLUE_CENTER_Y = 48.0;     // Blue base zone center Y
    public static final double BASE_ZONE_SIZE = 24.0;              // Base zone square size, inches

    // ==== Launch zones & lines (inches) ====
    public static final double HUMAN_LAUNCH_ZONE_CENTER_X = 0.0;   // Human-side launch zone center X
    public static final double HUMAN_LAUNCH_ZONE_CENTER_Y = 24.0;  // Human-side launch zone center Y
    public static final double TARGET_LAUNCH_ZONE_CENTER_X = 0.0;  // Target-side launch zone center X
    public static final double TARGET_LAUNCH_ZONE_CENTER_Y = 120.0; // Target-side launch zone center Y

    public static final double HUMAN_LAUNCH_LINE_Y  = 24.0;        // Y coordinate of human launch line
    public static final double TARGET_LAUNCH_LINE_Y = 120.0;       // Y coordinate of target launch line

    // ==== Initial artifact groups (rows share Y center) ====
    public static final double ARTIFACT_ROW_Y = 108.0;             // Common Y for the starting artifact rows
    public static final double GPP_CENTER_X = -48.0;               // Center X for GPP pattern row
    public static final double PGP_CENTER_X = 0.0;                 // Center X for PGP pattern row
    public static final double PPG_CENTER_X = 48.0;                // Center X for PPG pattern row
    public static final double ARTIFACT_SPACING = 6.0;             // Spacing between artifacts along +X, inches

    // ==== Odometry fusion parameters ====
    public static final double VISION_WEIGHT = 0.25;                    // Blend weight for AprilTag pose correction (0..1)
    public static final double MAX_VISION_CORRECTION_DISTANCE = 8.0;    // Max single-step vision correction, inches
    public static final double POSE_FILTER_STRENGTH = 0.3;              // Exponential smoothing factor for pose updates (0..1)
    public static final double IMU_HEADING_OFFSET_DEG = 0.0;            // Global heading offset applied to IMU yaw, degrees

    private OdometryConfig() { /* no instances */ }
}
