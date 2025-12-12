package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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
 *       • Computes planar distance from the robot pose to the alliance goal
 *         tag using Limelight botpose (MT2 preferred) scaled by
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
 */
public class LimelightTargetProvider implements VisionTargetProvider {
    private static final int OBELISK_CONFIRM_FRAMES = 2;
    private static final long OBELISK_STALE_MS = 1500L;

    private final Limelight3A limelight;
    private final Supplier<Alliance> allianceSupplier;

    private Integer latchedObeliskId = null;
    private Integer pendingObeliskId = null;
    private int pendingObeliskCount = 0;
    private long lastObeliskUpdateMs = 0L;
    private Long lastLatchedTimestampMs = null;

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
    public int getBestVisibleTagId() {
        TargetSnapshot snap = snapshot();
        return snap.bestVisibleId;
    }

    @Override
    public double getHeadingErrorDeg() {
        TargetSnapshot snap = snapshot();
        if (!snap.hasGoalTag || snap.result == null) return Double.NaN;
        return snap.result.getTx();
    }

    @Override
    public double getDistanceMeters() {
        TargetSnapshot snap = snapshot();
        if (!snap.hasGoalTag || snap.result == null) return Double.NaN;

        Pose3D pose = selectPose(snap.result);
        if (pose == null || pose.getPosition() == null) return Double.NaN;

        Alliance alliance = allianceSupplier != null ? allianceSupplier.get() : Alliance.BLUE;
        double goalX = VisionConfig.goalXMeters(alliance);
        double goalY = VisionConfig.goalYMeters(alliance);
        double dx = goalX - pose.getPosition().x;
        double dy = goalY - pose.getPosition().y;
        double distance = Math.hypot(dx, dy);

        return distance * VisionConfig.LIMELIGHT_RANGE_SCALE;
    }

    /** Debug helper: returns the alliance goal ID associated with this provider. */
    public int getAllianceGoalId() {
        Alliance alliance = allianceSupplier != null ? allianceSupplier.get() : Alliance.BLUE;
        return VisionConfig.goalTagIdForAlliance(alliance);
    }

    /** Debug helper: returns the latest visible tag IDs (empty list when none). */
    public List<Integer> getVisibleTagIds() {
        TargetSnapshot snap = snapshot();
        List<Integer> ids = new ArrayList<>();
        for (FiducialObservation obs : snap.observations) {
            ids.add(obs.id);
        }
        return ids;
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
        List<FiducialObservation> observations = extractFiducials(result);
        boolean resultValid = result != null && result.isValid();

        Alliance alliance = allianceSupplier != null ? allianceSupplier.get() : Alliance.BLUE;
        int allianceGoalId = VisionConfig.goalTagIdForAlliance(alliance);

        boolean hasGoal = resultValid && containsId(observations, allianceGoalId);
        int bestId = (resultValid && !observations.isEmpty()) ? observations.get(0).id : -1;
        if (hasGoal) {
            bestId = allianceGoalId;
        }

        return new TargetSnapshot(result, resultValid, hasGoal, allianceGoalId, bestId, observations);
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

    private boolean containsId(List<FiducialObservation> observations, int id) {
        for (FiducialObservation obs : observations) {
            if (obs.id == id) return true;
        }
        return false;
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
        final boolean hasGoalTag;
        final int allianceGoalId;
        final int bestVisibleId;
        final List<FiducialObservation> observations;

        TargetSnapshot(LLResult result,
                       boolean resultValid,
                       boolean hasGoalTag,
                       int allianceGoalId,
                       int bestVisibleId,
                       List<FiducialObservation> observations) {
            this.result = result;
            this.resultValid = resultValid;
            this.hasGoalTag = hasGoalTag;
            this.allianceGoalId = allianceGoalId;
            this.bestVisibleId = bestVisibleId;
            this.observations = observations;
        }
    }
}
