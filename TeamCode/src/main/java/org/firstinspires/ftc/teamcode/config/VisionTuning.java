/*
 * FILE: VisionTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Hold vision calibration values—currently the AprilTag range scale—so both
 *     TeleOp and Auto initialize the camera with the same correction factor.
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

    public static double RANGE_SCALE = 0.03;
}
