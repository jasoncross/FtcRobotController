package org.firstinspires.ftc.teamcode.vision;

/*
 * FILE: VisionTargetProvider.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/
 *
 * PURPOSE
 *   - Minimal heading + distance abstraction for goal-targeting sources so
 *     TeleOp and Autonomous logic can consume Limelight or legacy webcam
 *     pipelines through a shared interface.
 *
 * METHODS
 *   - hasTarget()
 *       • True when the underlying vision source currently tracks a goal tag.
 *   - getHeadingErrorDeg()
 *       • Signed left/right error to the scoring target in degrees (+ right,
 *         – left). Returns NaN when no target is available.
 *   - getDistanceMeters()
 *       • Range from the robot to the scoring target in meters. Returns NaN
 *         when no target is available.
 *
 * CHANGES (2025-12-11): Introduced the unified target abstraction to decouple
 *                       Limelight and legacy webcam pipelines ahead of vision
 *                       source routing.
 */
public interface VisionTargetProvider {
    boolean hasTarget();

    double getHeadingErrorDeg();

    double getDistanceMeters();
}
