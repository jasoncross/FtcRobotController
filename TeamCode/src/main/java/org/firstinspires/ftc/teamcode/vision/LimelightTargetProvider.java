package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.utils.ObeliskSignal;

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
 *       • True when the latest Limelight result is valid.
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
    private final Limelight3A limelight;
    private final Supplier<Alliance> allianceSupplier;

    public LimelightTargetProvider(Limelight3A limelight, Supplier<Alliance> allianceSupplier) {
        this.limelight = limelight;
        this.allianceSupplier = allianceSupplier;
    }

    @Override
    public boolean hasTarget() {
        LLResult result = latest();
        return result != null && result.isValid();
    }

    @Override
    public double getHeadingErrorDeg() {
        LLResult result = latest();
        return (result != null && result.isValid()) ? result.getTx() : Double.NaN;
    }

    @Override
    public double getDistanceMeters() {
        LLResult result = latest();
        if (result == null || !result.isValid()) return Double.NaN;

        Pose3D pose = selectPose(result);
        if (pose == null || pose.getPosition() == null) return Double.NaN;

        Alliance alliance = allianceSupplier != null ? allianceSupplier.get() : Alliance.BLUE;
        double goalX = VisionConfig.goalXMeters(alliance);
        double goalY = VisionConfig.goalYMeters(alliance);
        double dx = goalX - pose.getPosition().x;
        double dy = goalY - pose.getPosition().y;
        double distance = Math.hypot(dx, dy);

        return distance * VisionConfig.LIMELIGHT_RANGE_SCALE;
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
            if (result != null && result.isValid()) {
                return result;
            }
            return null;
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
        if (result == null) return;

        Integer fiducial = readSingleId(result, "getFiducialId");
        if (fiducial == null) fiducial = readSingleId(result, "getTid");
        if (fiducial == null) fiducial = readSingleId(result, "getTargetId");
        if (fiducial != null) {
            ObeliskSignal.updateFromTagId(fiducial);
        }

        int[] idArray = readIntArray(result, "getFiducialIds", "getTargetIds", "getTidList");
        if (idArray != null) {
            for (int id : idArray) {
                ObeliskSignal.updateFromTagId(id);
            }
        }

        double[] doubleArray = readDoubleArray(result, "getFiducialIds", "getTargetIds", "getTidList");
        if (doubleArray != null) {
            for (double id : doubleArray) {
                ObeliskSignal.updateFromTagId((int) Math.round(id));
            }
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

    private Integer readSingleId(LLResult result, String method) {
        try {
            Object value = result.getClass().getMethod(method).invoke(result);
            if (value instanceof Number) {
                return ((Number) value).intValue();
            }
        } catch (Throwable ignored) { }
        return null;
    }

    private int[] readIntArray(LLResult result, String... methods) {
        for (String method : methods) {
            try {
                Object value = result.getClass().getMethod(method).invoke(result);
                if (value instanceof int[]) {
                    return (int[]) value;
                }
            } catch (Throwable ignored) { }
        }
        return null;
    }

    private double[] readDoubleArray(LLResult result, String... methods) {
        for (String method : methods) {
            try {
                Object value = result.getClass().getMethod(method).invoke(result);
                if (value instanceof double[]) {
                    return (double[]) value;
                }
            } catch (Throwable ignored) { }
        }
        return null;
    }
}
