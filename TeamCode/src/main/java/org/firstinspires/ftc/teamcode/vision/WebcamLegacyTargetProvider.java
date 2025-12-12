package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.vision.VisionAprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.Supplier;

/*
 * LEGACY: Used only when vision is configured to webcam mode (P480/P720).
 * Limelight is the default and preferred implementation.
 *
 * FILE: WebcamLegacyTargetProvider.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/
 *
 * PURPOSE
 *   - Provide a VisionTargetProvider wrapper around the existing VisionPortal
 *     AprilTag pipeline so legacy webcam builds continue to compile while the
 *     Limelight becomes the default source.
 *
 * METHODS
 *   - hasTarget()
 *       • True when the configured alliance goal tag is visible in the webcam
 *         pipeline.
 *   - getHeadingErrorDeg()
 *       • Returns AprilTag bearing for the alliance goal or NaN when absent.
 *   - getDistanceMeters()
 *       • Returns AprilTag range (meters) for the alliance goal or NaN when
 *         absent.
 *
 * CHANGES (2025-12-11): Added the legacy wrapper so existing webcam pipelines
 *                       can satisfy the new target-provider abstraction without
 *                       changing call sites.
 * CHANGES (2025-12-12): Switched to SDK AprilTagDetection type to restore
 *                       compatibility after refactors.
 */
public class WebcamLegacyTargetProvider implements VisionTargetProvider {
    private final VisionAprilTag visionAprilTag;
    private final Supplier<Alliance> allianceSupplier;

    public WebcamLegacyTargetProvider(VisionAprilTag visionAprilTag, Supplier<Alliance> allianceSupplier) {
        this.visionAprilTag = visionAprilTag;
        this.allianceSupplier = allianceSupplier;
    }

    @Override
    public boolean hasTarget() {
        if (visionAprilTag == null) return false;
        return visionAprilTag.hasAnyGoalTagForTarget(goalId());
    }

    @Override
    public double getHeadingErrorDeg() {
        AprilTagDetection det = goalDetection();
        return det != null ? det.ftcPose.bearing : Double.NaN;
    }

    @Override
    public double getDistanceMeters() {
        AprilTagDetection det = goalDetection();
        return det != null ? det.ftcPose.range : Double.NaN;
    }

    private AprilTagDetection goalDetection() {
        if (visionAprilTag == null) return null;
        AprilTagDetection det = visionAprilTag.getDetectionFor(goalId());
        if (det == null) {
            det = visionAprilTag.getNearestGoalTag();
        }
        return det;
    }

    private int goalId() {
        Alliance alliance = allianceSupplier != null ? allianceSupplier.get() : Alliance.BLUE;
        return VisionConfig.goalTagIdForAlliance(alliance);
    }
}
