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
 *   - isGoalDetectedRaw()/isGoalDetectedSmoothed()
 *       • True when the alliance goal tag ID appears in the most recent frame
 *         (raw) and after any hysteresis/hold logic (smoothed), independent of
 *         heading validity.
 *   - isGoalAimValid()
 *       • True only when the heading sample is finite/usable for aim control.
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
 *   - getSmoothedHeadingErrorDeg()
 *       • Optional hysteresis-friendly heading sample that may hold the last
 *         known tx for a grace window. Defaults to the raw heading when
 *         unimplemented.
 *   - getDistanceMeters()
 *       • Range from the robot to the scoring target in meters. Returns NaN
 *         when no target is available.
 *   - isGoalVisibleRaw()
 *       • True when the scoring tag is present in the most recent frame.
 *   - isGoalVisibleSmoothed()
 *       • True when the provider still considers the goal acquired after
 *         applying any loss hysteresis/hold logic.
 *   - getGoalLostFrames()
 *       • Count of consecutive frames without the goal tag (smoothed)
 *         visibility.
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
 * CHANGES (2025-12-21): Added separate goal-detected vs. aim-valid accessors
 *                       so auto-aim enablement can gate on fiducial presence
 *                       plus finite heading instead of a single tagVisible flag.
 */
public interface VisionTargetProvider {
    boolean hasTarget();

    boolean hasGoalTarget();

    boolean hasAnyTarget();

    /** True when the alliance goal tag ID appears in the latest frame (no gating on tx validity). */
    default boolean isGoalDetectedRaw() { return hasGoalTarget(); }

    /** True when the alliance goal tag survives any acquire/loss hysteresis (no gating on tx validity). */
    default boolean isGoalDetectedSmoothed() { return isGoalDetectedRaw(); }

    /** True only when a finite heading sample is available for aim control. */
    default boolean isGoalAimValid() { return hasGoalTarget(); }

    default int getLatchedObeliskId() { return -1; }

    default List<Integer> getVisibleObeliskIds() { return Collections.emptyList(); }

    int getBestVisibleTagId();

    double getHeadingErrorDeg();

    default Double getSmoothedHeadingErrorDeg() {
        double heading = getHeadingErrorDeg();
        return Double.isNaN(heading) ? null : heading;
    }

    double getDistanceMeters();

    default boolean isGoalVisibleRaw() { return isGoalDetectedRaw(); }

    default boolean isGoalVisibleSmoothed() { return isGoalDetectedSmoothed(); }

    default int getGoalLostFrames() { return 0; }

    default void ensureGoalAimMode(Alliance alliance) {}

    default void ensureObeliskObservationMode() {}

    default Integer getActivePipelineIndex() { return null; }
}
