package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.utils.ObeliskSignal;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

/*
 * FILE: LimelightTargetProvider.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/
 *
 * PURPOSE
 *   - Provide heading + distance outputs sourced from the Limelight 3A so
 *     subsystems can target the scoring goal through the VisionTargetProvider
 *     abstraction.
 *   - Select the correct alliance goal tag for range calculations using
 *     Limelight 3D pose (preferring MegaTag2 when available).
 *
 * METHODS
 *   - hasTarget()
 *       • True when the latest Limelight result is valid AND contains the
 *         alliance goal tag (20 blue / 24 red).
 *   - hasAnyTarget()
 *       • True when the latest Limelight result is valid for any tag.
 *   - getBestVisibleTagId()
 *       • Returns the fiducial ID currently backing tx/range, or -1.
 *   - getHeadingErrorDeg()
 *       • Returns Limelight tx (degrees, + right / – left) or NaN when no
 *         target is available.
 *   - getDistanceMeters()
 *       • Returns the alliance goal tag's forward (TZ) component from
 *         Limelight target pose in ROBOT SPACE scaled by
 *         VisionConfig.LIMELIGHT_RANGE_SCALE. Returns NaN when no pose is
 *         available.
 *
 * CHANGES (2025-12-11): Added the Limelight-backed VisionTargetProvider with
 *                       alliance-aware goal selection and botpose distance
 *                       calculations.
 * CHANGES (2025-12-11): Added obelisk latching via Limelight detections so
 *                       motif tags remain available when the webcam stack is
 *                       disabled.
 * CHANGES (2025-12-12): Updated pose field accessors to match current Limelight
 *                       Pose3D API fields for build compatibility.
 * CHANGES (2025-12-16): Added Limelight-only pipeline assertion helpers for
 *                       obelisk observation vs. goal aiming so Auto can force
 *                       the correct AprilTag mode when Limelight is the active
 *                       source without touching webcam fallbacks.
 * CHANGES (2025-12-16): Smoothed goal visibility + aim tx selection for
 *                       Limelight so single-frame dropouts no longer cause
 *                       Auto scan/aim flicker; aim tx now stays locked to the
 *                       alliance goal tag only, with telemetry exposing the
 *                       smoothed state.
 */
public class LimelightTargetProvider implements VisionTargetProvider {
    private static final int OBELISK_CONFIRM_FRAMES = 2;
    private static final long OBELISK_STALE_MS = 1500L;
    private static final long AIM_LOCK_STALE_MS = 250L;          // How long to retain a goal lock after loss
    private static final long GOAL_VISIBLE_HOLD_MS = 250L;       // Hold window for sticky goal visibility to mask single-frame dropouts
    private static final int GOAL_MISSING_FRAMES_TO_DROP = 3;    // Consecutive missing frames required before clearing sticky visibility
    private static final double AIM_SWITCH_TX_HYST_DEG = 2.0;    // Margin needed to switch aim target
    private static final long PIPELINE_REASSERT_MS = 750L;       // Minimum gap between repeated pipeline assertions

    private final Limelight3A limelight;
    private final Supplier<Alliance> allianceSupplier;

    private Integer latchedObeliskId = null;
    private Integer pendingObeliskId = null;
    private int pendingObeliskCount = 0;
    private long lastObeliskUpdateMs = 0L;
    private Long lastLatchedTimestampMs = null;
    private DistanceEstimate lastDistanceEstimate = DistanceEstimate.empty();
    private int lockedAimTagId = -1;
    private long lockedAimLastSeenMs = 0L;
    private Double lockedAimLastTx = null;
    private int goalMissingFrames = 0;
    private Integer activePipelineIndex = null;
    private long lastPipelineCommandMs = 0L;

    public LimelightTargetProvider(Limelight3A limelight, Supplier<Alliance> allianceSupplier) {
        this.limelight = limelight;
        this.allianceSupplier = allianceSupplier;
    }

    @Override
    public boolean hasTarget() { return hasGoalTarget(); }

    @Override
    public boolean hasGoalTarget() {
        TargetSnapshot snap = snapshot();
        return snap.hasGoalTag;
    }

    @Override
    public boolean hasAnyTarget() {
        TargetSnapshot snap = snapshot();
        return snap.resultValid;
    }

    @Override
    public int getLatchedObeliskId() { return latchedObeliskId == null ? -1 : latchedObeliskId; }

    @Override
    public List<Integer> getVisibleObeliskIds() {
        TargetSnapshot snap = snapshot();
        List<Integer> obelisks = new ArrayList<>();
        for (FiducialObservation obs : snap.observations) {
            if (obs.id >= 21 && obs.id <= 23) {
                obelisks.add(obs.id);
            }
        }
        return obelisks;
    }

    @Override
    public int getBestVisibleTagId() {
        TargetSnapshot snap = snapshot();
        return snap.hasGoalTag ? snap.allianceGoalId : -1;
    }

    @Override
    public double getHeadingErrorDeg() {
        TargetSnapshot snap = snapshot();
        if (!snap.hasGoalTag) return Double.NaN;
        return snap.aimTxDegUsed != null ? snap.aimTxDegUsed : Double.NaN;
    }

    @Override
    public double getDistanceMeters() {
        TargetSnapshot snap = snapshot();
        DistanceEstimate estimate = computeDistanceMeters(snap);
        lastDistanceEstimate = estimate;
        if (estimate.distanceMeters == null) return Double.NaN;
        return estimate.distanceMeters;
    }

    /** Latest distance breakdown for telemetry (robot-space TZ + optional field-space compare). */
    public DistanceEstimate getLastDistanceEstimate() { return lastDistanceEstimate; }

    /** Debug helper: returns the alliance goal ID associated with this provider. */
    public int getAllianceGoalId() {
        Alliance alliance = allianceSupplier != null ? allianceSupplier.get() : Alliance.BLUE;
        return VisionConfig.goalTagIdForAlliance(alliance);
    }

    @Override
    public void ensureGoalAimMode(Alliance alliance) {
        assertPipeline(VisionConfig.LimelightFusion.GOAL_AIM_PIPELINE_INDEX);
        startIfNeeded();
    }

    @Override
    public void ensureObeliskObservationMode() {
        assertPipeline(VisionConfig.LimelightFusion.OBELISK_PIPELINE_INDEX);
        startIfNeeded();
    }

    @Override
    public Integer getActivePipelineIndex() { return activePipelineIndex; }

    /** Debug helper: returns the latest visible tag IDs (empty list when none). */
    public List<Integer> getVisibleTagIds() {
        TargetSnapshot snap = snapshot();
        List<Integer> ids = new ArrayList<>();
        for (FiducialObservation obs : snap.observations) {
            ids.add(obs.id);
        }
        return ids;
    }

    /** Telemetry helper exposing the latest aim lock state and selected tx sample. */
    public AimTelemetry getAimTelemetry() {
        TargetSnapshot snap = snapshot();
        long now = System.currentTimeMillis();
        long ageMs = (lockedAimTagId < 0 || lockedAimLastSeenMs <= 0L)
                ? -1L
                : Math.max(0L, now - lockedAimLastSeenMs);

        List<Integer> ids = new ArrayList<>();
        for (FiducialObservation obs : snap.observations) {
            ids.add(obs.id);
        }

        return new AimTelemetry(
                ids,
                snap.goalSeenThisFrame,
                snap.hasGoalTag,
                goalMissingFrames,
                lockedAimTagId,
                snap.aimTxDegUsed,
                ageMs);
    }

    private LLResult latest() {
        LLResult result = fetchLatest();
        if (!isFresh(result)) return null;
        return result;
    }

    private LLResult fetchLatest() {
        if (limelight == null) return null;
        try {
            LLResult result = limelight.getLatestResult();
            latchObelisk(result);
            return result;
        } catch (Throwable ignored) { return null; }
    }

    private Pose3D selectPose(LLResult result) {
        if (result == null) return null;

        Pose3D pose = result.getBotpose_MT2();
        if (pose != null) return pose;

        return result.getBotpose();
    }

    private boolean isFresh(LLResult result) {
        if (result == null) return false;
        Long tsMs = readTimestampMs(result);
        if (tsMs == null) return true; // No timestamp available—assume current

        long ageMs = System.currentTimeMillis() - tsMs;
        return ageMs <= VisionConfig.LimelightFusion.MAX_AGE_MS;
    }

    private void latchObelisk(LLResult result) {
        if (result == null || !isFresh(result)) return;

        Long tsMs = readTimestampMs(result);
        if (tsMs != null) {
            if (lastLatchedTimestampMs != null && lastLatchedTimestampMs.equals(tsMs)) return;
            lastLatchedTimestampMs = tsMs;
        }

        List<FiducialObservation> obs = extractFiducials(result);
        FiducialObservation bestObelisk = selectBestObelisk(obs);
        long now = System.currentTimeMillis();

        if (bestObelisk != null) {
            lastObeliskUpdateMs = now;
            if (latchedObeliskId != null && latchedObeliskId == bestObelisk.id) {
                pendingObeliskId = bestObelisk.id;
                pendingObeliskCount = OBELISK_CONFIRM_FRAMES;
                return;
            }

            if (pendingObeliskId != null && pendingObeliskId == bestObelisk.id) {
                pendingObeliskCount++;
            } else {
                pendingObeliskId = bestObelisk.id;
                pendingObeliskCount = 1;
            }

            if (pendingObeliskCount >= OBELISK_CONFIRM_FRAMES) {
                latchedObeliskId = bestObelisk.id;
                pendingObeliskId = latchedObeliskId;
                pendingObeliskCount = OBELISK_CONFIRM_FRAMES;
                ObeliskSignal.updateFromTagId(latchedObeliskId);
            }
            return;
        }

        if (latchedObeliskId != null && (now - lastObeliskUpdateMs) > OBELISK_STALE_MS) {
            latchedObeliskId = null;
            pendingObeliskId = null;
            pendingObeliskCount = 0;
            ObeliskSignal.set(ObeliskSignal.Order.UNKNOWN);
        }
    }

    private Long readTimestampMs(LLResult result) {
        try {
            Object seconds = result.getClass().getMethod("getTimestampSeconds").invoke(result);
            if (seconds instanceof Number) {
                return (long) (((Number) seconds).doubleValue() * 1000.0);
            }
        } catch (Throwable ignored) { }

        try {
            Object millis = result.getClass().getMethod("getTimestampMs").invoke(result);
            if (millis instanceof Number) {
                return ((Number) millis).longValue();
            }
        } catch (Throwable ignored) { }

        return null;
    }

    private TargetSnapshot snapshot() {
        LLResult result = latest();
        long now = System.currentTimeMillis();
        List<FiducialObservation> observations = extractFiducials(result);
        boolean resultValid = result != null && result.isValid();

        Alliance alliance = allianceSupplier != null ? allianceSupplier.get() : Alliance.BLUE;
        int allianceGoalId = VisionConfig.goalTagIdForAlliance(alliance);

        FiducialObservation goalObs = selectBestGoalObservation(allianceGoalId, observations);
        updateAimLock(now, allianceGoalId, goalObs);

        boolean goalSeenThisFrame = resultValid && goalObs != null;
        goalMissingFrames = goalSeenThisFrame ? 0 : goalMissingFrames + 1;

        boolean goalLockFresh = lockedAimTagId == allianceGoalId
                && (now - lockedAimLastSeenMs) <= GOAL_VISIBLE_HOLD_MS;
        boolean hasGoalSmoothed = goalSeenThisFrame
                || (goalLockFresh && goalMissingFrames < GOAL_MISSING_FRAMES_TO_DROP);
        int bestId = (resultValid && !observations.isEmpty()) ? observations.get(0).id : -1;
        if (hasGoalSmoothed) {
            bestId = allianceGoalId;
        }

        Double aimTxDeg = null;
        if (goalSeenThisFrame && goalObs != null && goalObs.txDeg != null) {
            aimTxDeg = goalObs.txDeg;
        } else if (hasGoalSmoothed && lockedAimLastTx != null) {
            aimTxDeg = lockedAimLastTx;
        }

        return new TargetSnapshot(
                result,
                resultValid,
                goalSeenThisFrame,
                hasGoalSmoothed,
                allianceGoalId,
                bestId,
                observations,
                goalObs,
                aimTxDeg,
                now);
    }

    private void updateAimLock(long nowMs, int allianceGoalId, FiducialObservation goalObs) {
        // Prefer the alliance goal immediately when seen.
        if (goalObs != null) {
            lockedAimTagId = allianceGoalId;
            lockedAimLastSeenMs = nowMs;
            lockedAimLastTx = goalObs.txDeg != null ? goalObs.txDeg : lockedAimLastTx;
            return;
        }

        // Expire the lock once the hold window has elapsed to avoid clinging to stale aim data.
        if (lockedAimTagId == allianceGoalId && (nowMs - lockedAimLastSeenMs) > AIM_LOCK_STALE_MS) {
            lockedAimTagId = -1;
            lockedAimLastTx = null;
        }
    }

    private FiducialObservation selectBestGoalObservation(int allianceGoalId, List<FiducialObservation> observations) {
        FiducialObservation best = null;

        for (FiducialObservation obs : observations) {
            if (obs.id != allianceGoalId) continue;

            if (best == null) {
                best = obs;
                continue;
            }

            Double obsAbs = obs.txDeg != null ? Math.abs(obs.txDeg) : null;
            Double bestAbs = best.txDeg != null ? Math.abs(best.txDeg) : null;

            if (obsAbs == null) continue;
            if (bestAbs == null || obsAbs + AIM_SWITCH_TX_HYST_DEG < bestAbs) {
                best = obs;
            }
        }

        return best;
    }

    private DistanceEstimate computeDistanceMeters(TargetSnapshot snap) {
        if (snap == null || !snap.hasGoalTag || snap.result == null) {
            return DistanceEstimate.empty();
        }

        if (!snap.goalSeenThisFrame) {
            boolean lockFresh = lockedAimTagId == snap.allianceGoalId
                    && (snap.snapshotMs - lockedAimLastSeenMs) <= GOAL_VISIBLE_HOLD_MS;
            if (lockFresh && lastDistanceEstimate.distanceMeters != null) {
                return lastDistanceEstimate;
            }
            return DistanceEstimate.empty();
        }

        Pose3D robotSpacePose = selectTargetPoseRobotSpace(snap.result, snap.allianceGoalId);
        Double forwardMeters = extractForwardMeters(robotSpacePose);
        Double scaledMeters = (forwardMeters != null) ? forwardMeters * VisionConfig.LIMELIGHT_RANGE_SCALE : null;

        Double fieldMeters = null;
        Pose3D botpose = selectPose(snap.result);
        if (botpose != null && botpose.getPosition() != null) {
            Alliance alliance = allianceSupplier != null ? allianceSupplier.get() : Alliance.BLUE;
            double goalX = VisionConfig.goalXMeters(alliance);
            double goalY = VisionConfig.goalYMeters(alliance);
            double dx = goalX - botpose.getPosition().x;
            double dy = goalY - botpose.getPosition().y;
            fieldMeters = Math.hypot(dx, dy) * VisionConfig.LIMELIGHT_RANGE_SCALE;
        }

        return new DistanceEstimate(forwardMeters, scaledMeters, fieldMeters);
    }

    private void assertPipeline(int pipelineIndex) {
        if (limelight == null) return;
        long now = System.currentTimeMillis();
        if (activePipelineIndex != null && activePipelineIndex == pipelineIndex && (now - lastPipelineCommandMs) < PIPELINE_REASSERT_MS) {
            return;
        }

        boolean changed = false;
        try {
            limelight.pipelineSwitch(pipelineIndex);
            changed = true;
        } catch (Throwable ignored) { }

        if (!changed) {
            try {
                limelight.getClass().getMethod("setPipelineIndex", int.class)
                        .invoke(limelight, pipelineIndex);
                changed = true;
            } catch (Throwable ignored) { }
        }

        if (changed) {
            activePipelineIndex = pipelineIndex;
            lastPipelineCommandMs = now;
        }
    }

    private void startIfNeeded() {
        if (limelight == null) return;
        try { limelight.start(); } catch (Throwable ignored) { }
    }

    private Pose3D selectTargetPoseRobotSpace(LLResult result, int allianceGoalId) {
        if (result == null) return null;

        // First preference: fiducial entry matching the alliance goal ID.
        Pose3D fiducialPose = findFiducialPose(result, allianceGoalId);
        if (fiducialPose != null) {
            return fiducialPose;
        }

        // Fallback: whatever pose the result exposes as the primary target pose.
        return readPose(result, "getTargetPose_RobotSpace", "getTargetPoseRobotSpace", "getTargetPose_robotSpace");
    }

    private Pose3D findFiducialPose(LLResult result, int allianceGoalId) {
        List<?> fiducialResults = readList(result, "getFiducialResults");
        if (fiducialResults == null) return null;

        for (Object entry : fiducialResults) {
            Integer id = readSingleId(entry, "getFiducialId", "getTid", "getTargetId");
            if (id == null || id != allianceGoalId) continue;

            Pose3D pose = readPose(entry,
                    "getTargetPose_RobotSpace",
                    "getTargetPoseRobotSpace",
                    "getTargetPose_robotSpace",
                    "getRobotPoseTargetSpace",
                    "getRobotSpacePose");
            if (pose != null) {
                return pose;
            }
        }

        return null;
    }

    private Double extractForwardMeters(Pose3D pose) {
        if (pose == null || pose.getPosition() == null) return null;
        double tz = pose.getPosition().z;
        if (Double.isNaN(tz) || !Double.isFinite(tz)) return null;
        return tz;
    }

    private List<FiducialObservation> extractFiducials(LLResult result) {
        if (result == null) return Collections.emptyList();

        List<FiducialObservation> out = new ArrayList<>();

        List<?> fiducialResults = readList(result, "getFiducialResults");
        if (fiducialResults != null) {
            for (Object entry : fiducialResults) {
                Integer id = readSingleId(entry, "getFiducialId", "getTid", "getTargetId");
                Double tx = readSingleDouble(entry, "getTx", "getTxDegrees", "getTxRadians");
                if (id != null) {
                    out.add(new FiducialObservation(id, tx));
                }
            }
        }

        if (out.isEmpty()) {
            int[] idArray = readIntArray(result, "getFiducialResults", "getFiducialIds", "getTargetIds", "getTidList");
            if (idArray != null) {
                for (int id : idArray) {
                    out.add(new FiducialObservation(id, null));
                }
            }
        }

        if (out.isEmpty()) {
            double[] doubleArray = readDoubleArray(result, "getFiducialResults", "getFiducialIds", "getTargetIds", "getTidList");
            if (doubleArray != null) {
                for (double id : doubleArray) {
                    out.add(new FiducialObservation((int) Math.round(id), null));
                }
            }
        }

        if (out.isEmpty()) {
            Integer fiducial = readSingleId(result, "getFiducialId", "getTid", "getTargetId");
            if (fiducial != null) {
                out.add(new FiducialObservation(fiducial, null));
            }
        }

        return out;
    }

    private FiducialObservation selectBestObelisk(List<FiducialObservation> observations) {
        FiducialObservation best = null;

        for (FiducialObservation obs : observations) {
            if (obs.id < 21 || obs.id > 23) continue;

            if (best == null) {
                best = obs;
                continue;
            }

            if (best.txDeg == null && obs.txDeg != null) {
                best = obs;
                continue;
            }

            if (obs.txDeg != null && best.txDeg != null) {
                if (Math.abs(obs.txDeg) < Math.abs(best.txDeg)) {
                    best = obs;
                }
            }
        }

        return best;
    }

    private Integer readSingleId(Object owner, String... methods) {
        for (String method : methods) {
            try {
                Object value = owner.getClass().getMethod(method).invoke(owner);
                if (value instanceof Number) {
                    return ((Number) value).intValue();
                }
            } catch (Throwable ignored) { }
        }
        return null;
    }

    private Pose3D readPose(Object owner, String... methods) {
        for (String method : methods) {
            try {
                Object value = owner.getClass().getMethod(method).invoke(owner);
                if (value instanceof Pose3D) {
                    return (Pose3D) value;
                }
                if (value instanceof double[]) {
                    double[] arr = (double[]) value;
                    if (arr.length >= 3) {
                        return new Pose3D(
                                new Position(DistanceUnit.METER, arr[0], arr[1], arr[2], 0L),
                                new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0L));
                    }
                }
            } catch (Throwable ignored) { }
        }
        return null;
    }

    private int[] readIntArray(Object owner, String... methods) {
        for (String method : methods) {
            try {
                Object value = owner.getClass().getMethod(method).invoke(owner);
                if (value instanceof List) {
                    List<?> list = (List<?>) value;
                    int[] arr = new int[list.size()];
                    for (int i = 0; i < list.size(); i++) {
                        Object v = list.get(i);
                        arr[i] = (v instanceof Number) ? ((Number) v).intValue() : 0;
                    }
                    return arr;
                }
                if (value instanceof int[]) {
                    return (int[]) value;
                }
            } catch (Throwable ignored) { }
        }
        return null;
    }

    private double[] readDoubleArray(Object owner, String... methods) {
        for (String method : methods) {
            try {
                Object value = owner.getClass().getMethod(method).invoke(owner);
                if (value instanceof List) {
                    List<?> list = (List<?>) value;
                    double[] arr = new double[list.size()];
                    for (int i = 0; i < list.size(); i++) {
                        Object v = list.get(i);
                        arr[i] = (v instanceof Number) ? ((Number) v).doubleValue() : Double.NaN;
                    }
                    return arr;
                }
                if (value instanceof double[]) {
                    return (double[]) value;
                }
            } catch (Throwable ignored) { }
        }
        return null;
    }

    private List<?> readList(Object owner, String... methods) {
        for (String method : methods) {
            try {
                Object value = owner.getClass().getMethod(method).invoke(owner);
                if (value instanceof List) {
                    return (List<?>) value;
                }
            } catch (Throwable ignored) { }
        }
        return null;
    }

    private Double readSingleDouble(Object owner, String... methods) {
        for (String method : methods) {
            try {
                Object value = owner.getClass().getMethod(method).invoke(owner);
                if (value instanceof Number) {
                    return ((Number) value).doubleValue();
                }
            } catch (Throwable ignored) { }
        }
        return null;
    }

    private static final class FiducialObservation {
        final int id;
        final Double txDeg;

        FiducialObservation(int id, Double txDeg) {
            this.id = id;
            this.txDeg = txDeg;
        }
    }

    private static final class TargetSnapshot {
        final LLResult result;
        final boolean resultValid;
        final boolean goalSeenThisFrame;
        final boolean hasGoalTag;
        final int allianceGoalId;
        final int bestVisibleId;
        final List<FiducialObservation> observations;
        final FiducialObservation goalObservation;
        final Double aimTxDegUsed;
        final long snapshotMs;

        TargetSnapshot(LLResult result,
                       boolean resultValid,
                       boolean goalSeenThisFrame,
                       boolean hasGoalTag,
                       int allianceGoalId,
                       int bestVisibleId,
                       List<FiducialObservation> observations,
                       FiducialObservation goalObservation,
                       Double aimTxDegUsed,
                       long snapshotMs) {
            this.result = result;
            this.resultValid = resultValid;
            this.goalSeenThisFrame = goalSeenThisFrame;
            this.hasGoalTag = hasGoalTag;
            this.allianceGoalId = allianceGoalId;
            this.bestVisibleId = bestVisibleId;
            this.observations = observations;
            this.goalObservation = goalObservation;
            this.aimTxDegUsed = aimTxDegUsed;
            this.snapshotMs = snapshotMs;
        }
    }

    /** Telemetry bundle for aim lock state. */
    public static final class AimTelemetry {
        public final List<Integer> visibleIds;
        public final boolean goalSeenThisFrame;
        public final boolean goalVisibleSmoothed;
        public final int goalMissingFrames;
        public final int lockedAimTagId;
        public final Double aimTxDeg;
        public final long lockAgeMs;

        AimTelemetry(List<Integer> visibleIds,
                     boolean goalSeenThisFrame,
                     boolean goalVisibleSmoothed,
                     int goalMissingFrames,
                     int lockedAimTagId,
                     Double aimTxDeg,
                     long lockAgeMs) {
            this.visibleIds = visibleIds;
            this.goalSeenThisFrame = goalSeenThisFrame;
            this.goalVisibleSmoothed = goalVisibleSmoothed;
            this.goalMissingFrames = goalMissingFrames;
            this.lockedAimTagId = lockedAimTagId;
            this.aimTxDeg = aimTxDeg;
            this.lockAgeMs = lockAgeMs;
        }
    }

    /** Struct capturing the latest Limelight distance breakdown for telemetry. */
    public static final class DistanceEstimate {
        public final Double targetForwardMeters;
        public final Double distanceMeters;
        public final Double fieldDistanceMeters;

        DistanceEstimate(Double targetForwardMeters, Double distanceMeters, Double fieldDistanceMeters) {
            this.targetForwardMeters = targetForwardMeters;
            this.distanceMeters = distanceMeters;
            this.fieldDistanceMeters = fieldDistanceMeters;
        }

        static DistanceEstimate empty() { return new DistanceEstimate(null, null, null); }
    }
}
