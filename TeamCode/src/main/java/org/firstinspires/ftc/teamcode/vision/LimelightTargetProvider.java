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
 * CHANGES (2025-12-16): Added smoothed goal visibility + held tx outputs with
 *                       configurable acquire/loss frames + hold timers so AUTO
 *                       ignores single-frame dropouts and keeps scanning stable.
 * CHANGES (2025-12-16): Shortened goal tx hold to reduce stale headings while
 *                       the robot is rotating quickly between sightlines.
 * CHANGES (2025-12-17): Enforced per-fiducial goal locking for aim/distance,
 *                       removed global tx fallbacks, and added tunables for
 *                       lock stale, hysteresis, and confirmation frames so
 *                       aim control only follows the alliance goal tag.
 */
public class LimelightTargetProvider implements VisionTargetProvider {
    private static final int OBELISK_CONFIRM_FRAMES = 2;
    private static final long OBELISK_STALE_MS = 1500L;
    private static final long PIPELINE_REASSERT_MS = 750L;       // Minimum gap between repeated pipeline assertions
    private static final int GOAL_ACQUIRE_FRAMES = VisionConfig.AimLock.AIM_SWITCH_CONFIRM_FRAMES; // Require N consecutive hits before declaring visible
    private static final int GOAL_LOST_FRAMES = 6;               // Frames without the goal before clearing smoothed state
    private static final long GOAL_HOLD_MS = VisionConfig.AimLock.AIM_LOCK_STALE_MS; // Hold last tx for this many ms after loss (short to avoid stale tx while turning fast)

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
    private Double pendingLockedTx = null;
    private int pendingLockedTxFrames = 0;
    private Integer activePipelineIndex = null;
    private long lastPipelineCommandMs = 0L;
    private int goalLostFrames = 0;
    private int goalSeenFrames = 0;
    private long lastGoalSeenMs = 0L;
    private boolean lastGoalVisibleRaw = false;
    private boolean lastGoalVisibleSmoothed = false;
    private Long lastProcessedFrameTimestampMs = null;
    private TargetSnapshot cachedSnapshot = null;
    private long lastSnapshotWallMs = 0L;

    public LimelightTargetProvider(Limelight3A limelight, Supplier<Alliance> allianceSupplier) {
        this.limelight = limelight;
        this.allianceSupplier = allianceSupplier;
    }

    @Override
    public boolean hasTarget() { return hasGoalTarget(); }

    @Override
    public boolean hasGoalTarget() {
        TargetSnapshot snap = snapshot();
        return snap.goalVisibleRaw;
    }

    @Override
    public boolean isGoalVisibleRaw() { return hasGoalTarget(); }

    @Override
    public boolean isGoalVisibleSmoothed() {
        TargetSnapshot snap = snapshot();
        return snap.goalVisibleSmoothed;
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
        return snap.goalVisibleSmoothed ? snap.allianceGoalId : -1;
    }

    @Override
    public double getHeadingErrorDeg() {
        TargetSnapshot snap = snapshot();
        return snap.txLockedUsedDeg != null ? snap.txLockedUsedDeg : Double.NaN;
    }

    @Override
    public Double getSmoothedHeadingErrorDeg() {
        TargetSnapshot snap = snapshot();
        return snap.txLockedUsedDeg;
    }

    @Override
    public int getGoalLostFrames() { return goalLostFrames; }

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
        long ageMs = snap.lockAgeMs;

        List<Integer> ids = new ArrayList<>();
        for (FiducialObservation obs : snap.observations) {
            ids.add(obs.id);
        }

        return new AimTelemetry(ids,
                snap.goalVisibleRaw,
                snap.goalVisibleSmoothed,
                lockedAimTagId,
                snap.txLockedUsedDeg,
                ageMs,
                goalLostFrames,
                snap.lockFresh);
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
        long now = System.currentTimeMillis();
        if (cachedSnapshot != null && (now - lastSnapshotWallMs) < 10L) {
            return cachedSnapshot;
        }

        LLResult result = latest();
        Long frameTs = readTimestampMs(result);
        boolean newFrame = frameTs == null || !frameTs.equals(lastProcessedFrameTimestampMs);

        List<FiducialObservation> observations = extractFiducials(result);
        boolean resultValid = result != null && result.isValid();

        Alliance alliance = allianceSupplier != null ? allianceSupplier.get() : Alliance.BLUE;
        int allianceGoalId = VisionConfig.goalTagIdForAlliance(alliance);

        FiducialObservation goalObs = resultValid ? selectBestGoalObservation(allianceGoalId, observations) : null;
        updateAimLock(now, allianceGoalId, goalObs);

        boolean hasGoalRaw = resultValid && goalObs != null && goalObs.txDeg != null;
        boolean lockFresh = isLockFresh(now, allianceGoalId);
        updateGoalVisibility(hasGoalRaw, now, newFrame, lockFresh);

        boolean hasGoal = lastGoalVisibleRaw;
        boolean smoothedGoal = lastGoalVisibleSmoothed;
        int bestId = (!observations.isEmpty()) ? observations.get(0).id : -1;
        if (smoothedGoal) {
            bestId = allianceGoalId;
        }

        Double txLockedUsed = lockFresh ? lockedAimLastTx : null;
        long lockAge = (lockedAimLastSeenMs <= 0L || lockedAimTagId != allianceGoalId)
                ? -1L
                : Math.max(0L, now - lockedAimLastSeenMs);

        TargetSnapshot snap = new TargetSnapshot(result, resultValid, hasGoal, smoothedGoal, allianceGoalId,
                bestId, observations, goalObs, txLockedUsed, now, frameTs, lockFresh, lockAge);
        cachedSnapshot = snap;
        lastSnapshotWallMs = now;
        if (newFrame) {
            lastProcessedFrameTimestampMs = frameTs;
        }
        return snap;
    }

    /**
     * Hysteresis helper: require several consecutive detections before declaring the goal visible,
     * hold the last tx briefly after loss, and only drop smoothed visibility once both the hold
     * window and the lost-frame counter expire. This prevents AUTO scan/aim thrash when Limelight
     * misses a single frame.
     */
    private void updateGoalVisibility(boolean rawVisible, long nowMs, boolean newFrame, boolean lockFresh) {
        if (newFrame) {
            if (rawVisible) {
                goalSeenFrames = Math.min(GOAL_ACQUIRE_FRAMES, goalSeenFrames + 1);
                goalLostFrames = 0;
                lastGoalSeenMs = nowMs;
            } else {
                goalSeenFrames = 0;
                goalLostFrames = Math.min(goalLostFrames + 1, Integer.MAX_VALUE);
            }
            lastGoalVisibleRaw = rawVisible && goalSeenFrames >= GOAL_ACQUIRE_FRAMES;
        }

        boolean holdWindow = lastGoalSeenMs > 0
                && (nowMs - lastGoalSeenMs) < GOAL_HOLD_MS
                && goalLostFrames < GOAL_LOST_FRAMES;
        lastGoalVisibleSmoothed = lastGoalVisibleRaw || holdWindow || lockFresh;
    }

    private void updateAimLock(long nowMs, int allianceGoalId, FiducialObservation goalObs) {
        // Prefer the alliance goal immediately when seen.
        if (goalObs != null && goalObs.txDeg != null) {
            if (lockedAimTagId != allianceGoalId) {
                lockedAimTagId = allianceGoalId;
                lockedAimLastTx = goalObs.txDeg;
                pendingLockedTx = null;
                pendingLockedTxFrames = 0;
            } else {
                double currentAbs = (lockedAimLastTx != null) ? Math.abs(lockedAimLastTx) : Double.POSITIVE_INFINITY;
                double candidateAbs = Math.abs(goalObs.txDeg);
                if (lockedAimLastTx == null || candidateAbs + VisionConfig.AimLock.AIM_SWITCH_TX_HYST_DEG < currentAbs) {
                    if (pendingLockedTx != null && pendingLockedTx.equals(goalObs.txDeg)) {
                        pendingLockedTxFrames++;
                    } else {
                        pendingLockedTx = goalObs.txDeg;
                        pendingLockedTxFrames = 1;
                    }
                    if (pendingLockedTxFrames >= VisionConfig.AimLock.AIM_SWITCH_CONFIRM_FRAMES) {
                        lockedAimLastTx = pendingLockedTx;
                    }
                } else {
                    pendingLockedTx = null;
                    pendingLockedTxFrames = 0;
                    lockedAimLastTx = goalObs.txDeg;
                }
            }
            lockedAimLastSeenMs = nowMs;
            return;
        }

        pendingLockedTx = null;
        pendingLockedTxFrames = 0;

        // Expire the lock quickly when goal disappears.
        if (lockedAimTagId == allianceGoalId && (nowMs - lockedAimLastSeenMs) > VisionConfig.AimLock.AIM_LOCK_STALE_MS) {
            lockedAimTagId = -1;
            lockedAimLastTx = null;
        }
    }

    private boolean isLockFresh(long nowMs, int allianceGoalId) {
        if (lockedAimTagId != allianceGoalId || lockedAimLastTx == null || lockedAimLastSeenMs <= 0L) {
            return false;
        }
        return (nowMs - lockedAimLastSeenMs) <= VisionConfig.AimLock.AIM_LOCK_STALE_MS;
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
            if (bestAbs == null || obsAbs + VisionConfig.AimLock.AIM_SWITCH_TX_HYST_DEG < bestAbs) {
                best = obs;
            }
        }

        return best;
    }

    private DistanceEstimate computeDistanceMeters(TargetSnapshot snap) {
        if (snap == null || !snap.goalVisibleRaw || snap.result == null || snap.goalObservation == null) {
            return DistanceEstimate.empty();
        }

        Double forwardMeters = snap.goalObservation.forwardMeters;
        if (forwardMeters == null) {
            Pose3D robotSpacePose = selectTargetPoseRobotSpace(snap.result, snap.allianceGoalId);
            forwardMeters = extractForwardMeters(robotSpacePose);
        }
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
                Pose3D pose = readPose(entry,
                        "getTargetPose_RobotSpace",
                        "getTargetPoseRobotSpace",
                        "getTargetPose_robotSpace",
                        "getRobotPoseTargetSpace",
                        "getRobotSpacePose");
                Double forward = extractForwardMeters(pose);
                if (id != null) {
                    out.add(new FiducialObservation(id, tx, forward));
                }
            }
        }

        if (out.isEmpty()) {
            int[] idArray = readIntArray(result, "getFiducialResults", "getFiducialIds", "getTargetIds", "getTidList");
            if (idArray != null) {
                for (int id : idArray) {
                    out.add(new FiducialObservation(id, null, null));
                }
            }
        }

        if (out.isEmpty()) {
            double[] doubleArray = readDoubleArray(result, "getFiducialResults", "getFiducialIds", "getTargetIds", "getTidList");
            if (doubleArray != null) {
                for (double id : doubleArray) {
                    out.add(new FiducialObservation((int) Math.round(id), null, null));
                }
            }
        }

        if (out.isEmpty()) {
            Integer fiducial = readSingleId(result, "getFiducialId", "getTid", "getTargetId");
            if (fiducial != null) {
                out.add(new FiducialObservation(fiducial, null, null));
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
        final Double forwardMeters;

        FiducialObservation(int id, Double txDeg, Double forwardMeters) {
            this.id = id;
            this.txDeg = txDeg;
            this.forwardMeters = forwardMeters;
        }
    }

    private static final class TargetSnapshot {
        final LLResult result;
        final boolean resultValid;
        final boolean goalVisibleRaw;
        final boolean goalVisibleSmoothed;
        final int allianceGoalId;
        final int bestVisibleId;
        final List<FiducialObservation> observations;
        final FiducialObservation goalObservation;
        final Double txLockedUsedDeg;
        final long snapshotMs;
        final Long frameTimestampMs;
        final boolean lockFresh;
        final long lockAgeMs;

        TargetSnapshot(LLResult result,
                       boolean resultValid,
                       boolean goalVisibleRaw,
                       boolean goalVisibleSmoothed,
                       int allianceGoalId,
                       int bestVisibleId,
                       List<FiducialObservation> observations,
                        FiducialObservation goalObservation,
                       Double txLockedUsedDeg,
                       long snapshotMs,
                       Long frameTimestampMs,
                       boolean lockFresh,
                       long lockAgeMs) {
            this.result = result;
            this.resultValid = resultValid;
            this.goalVisibleRaw = goalVisibleRaw;
            this.goalVisibleSmoothed = goalVisibleSmoothed;
            this.allianceGoalId = allianceGoalId;
            this.bestVisibleId = bestVisibleId;
            this.observations = observations;
            this.goalObservation = goalObservation;
            this.txLockedUsedDeg = txLockedUsedDeg;
            this.snapshotMs = snapshotMs;
            this.frameTimestampMs = frameTimestampMs;
            this.lockFresh = lockFresh;
            this.lockAgeMs = lockAgeMs;
        }
    }

    /** Telemetry bundle for aim lock state. */
    public static final class AimTelemetry {
        public final List<Integer> visibleIds;
        public final boolean goalVisible;
        public final boolean goalVisibleSmoothed;
        public final int lockedAimTagId;
        public final Double txLockedUsedDeg;
        public final long lockAgeMs;
        public final int goalLostFrames;
        public final boolean lockFresh;

        AimTelemetry(List<Integer> visibleIds,
                     boolean goalVisible,
                     boolean goalVisibleSmoothed,
                     int lockedAimTagId,
                     Double txLockedUsedDeg,
                     long lockAgeMs,
                     int goalLostFrames,
                     boolean lockFresh) {
            this.visibleIds = visibleIds;
            this.goalVisible = goalVisible;
            this.goalVisibleSmoothed = goalVisibleSmoothed;
            this.lockedAimTagId = lockedAimTagId;
            this.txLockedUsedDeg = txLockedUsedDeg;
            this.lockAgeMs = lockAgeMs;
            this.goalLostFrames = goalLostFrames;
            this.lockFresh = lockFresh;
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
