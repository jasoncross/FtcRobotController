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
