package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.Alliance;

import java.util.Collections;
import java.util.List;

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
 *       • True when the underlying vision source currently tracks the alliance goal tag.
 *   - hasGoalTarget()
 *       • Alias for hasTarget(); explicit name for callers that should only react
 *         to the alliance scoring tag.
 *   - hasAnyTarget()
 *       • True when any valid AprilTag solve is present (goal OR other tags).
 *   - getLatchedObeliskId()
 *       • Returns the last latched obelisk ID (21–23) or -1 if none is present.
 *   - getVisibleObeliskIds()
 *       • Returns any currently visible obelisk IDs (empty list when none).
 *   - getBestVisibleTagId()
 *       • Returns the fiducial ID currently driving heading/distance or -1 when
 *         unknown.
 *   - getHeadingErrorDeg()
 *       • Signed left/right error to the scoring target in degrees (+ right,
 *         – left). Returns NaN when no target is available.
 *   - getDistanceMeters()
 *       • Range from the robot to the scoring target in meters. Returns NaN
 *         when no target is available.
 *   - ensureGoalAimMode(alliance)
 *       • Allows Limelight-backed implementations to assert the correct
 *         AprilTag pipeline for alliance-goal aiming; no-op for webcam.
 *   - ensureObeliskObservationMode()
 *       • Allows Limelight-backed implementations to assert the correct
 *         AprilTag pipeline while observing obelisk motifs; no-op for webcam.
 *
 * CHANGES (2025-12-11): Introduced the unified target abstraction to decouple
 *                       Limelight and legacy webcam pipelines ahead of vision
 *                       source routing.
 * CHANGES (2025-12-16): Added explicit alliance-goal vs. obelisk accessors and
 *                       Limelight-only mode assertion hooks so Auto can force
 *                       the correct pipeline without touching webcam fallbacks.
 */
public interface VisionTargetProvider {
    boolean hasTarget();

    boolean hasGoalTarget();

    boolean hasAnyTarget();

    default int getLatchedObeliskId() { return -1; }

    default List<Integer> getVisibleObeliskIds() { return Collections.emptyList(); }

    int getBestVisibleTagId();

    double getHeadingErrorDeg();

    double getDistanceMeters();

    default void ensureGoalAimMode(Alliance alliance) {}

    default void ensureObeliskObservationMode() {}

    default Integer getActivePipelineIndex() { return null; }
}
