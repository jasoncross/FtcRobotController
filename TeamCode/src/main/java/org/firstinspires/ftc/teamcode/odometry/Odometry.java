package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.OdometryConfig;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.drive.Drivebase;
import org.firstinspires.ftc.teamcode.vision.VisionAprilTag;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.List;
import java.util.Set;

import static java.lang.Math.*;

/*
 * FILE: Odometry.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/odometry/
 *
 * PURPOSE
 *   - Fuse drivetrain encoder deltas, IMU heading, and optional Limelight XY
 *     corrections into a stable field pose anchored at the robot center.
 *   - Publish a consistent coordinate frame for TeleOp, Auto, and dashboard
 *     drawing utilities without duplicating geometry math across callers.
 *
 * NOTES
 *   - Heading comes from Drivebase.heading() plus the configurable IMU offset.
 *   - Camera corrections blend position only (X/Y); yaw is always IMU-only.
 *   - Coordinate frame (FieldPose): inches, origin at field center, +Y toward
 *     the target wall, +X to the robot's right when facing the targets, heading
 *     0° facing +Y with +CCW rotation.
 *
 * CHANGES (2025-11-26): Corrected odometry sign conventions so +X is right and
 *                        +Y is toward the targets, normalized IMU heading to
 *                        the shared frame (0° facing +Y, +CCW), and reused the
 *                        same heading basis for vision pose fusion.
 * CHANGES (2025-11-25): Added goal-tag pose fusion that leverages config tag poses,
 *                        camera offsets, and pitch/yaw to compute the robot center
 *                        from detections, enabling Auto→TeleOp pose carryover.
 * CHANGES (2025-12-11): Switched to the FTC field-center frame (+Y toward targets),
 *                        removed webcam-based fusion, and added Limelight-only
 *                        XY blending with reacquire-friendly gating and bounded
 *                        correction steps.
 * CHANGES (2025-12-12): Updated Limelight pose accessors for current API field
 *                        members to restore build compatibility on center-frame
 *                        odometry.
 */
public class Odometry {

    private final Drivebase drive;
    private final Limelight3A limelight;

    private final double[] lastWheelInches = new double[4];
    private boolean initialized = false;
    private FieldPose pose = new FieldPose();
    private FieldPose wheelPose = new FieldPose();

    private long lastUpdateNanos = -1L;
    private long lastAcceptedVisionMs = -1L;
    private FieldPose lastVisionPose = null;
    private final CameraPoseGate cameraPoseGate = new CameraPoseGate();
    private CameraDebug cameraDebug = new CameraDebug();

    private static final double M_TO_IN = 39.37007874;
    private static final Set<Integer> OBELISK_TAGS = Set.of(21, 22, 23);

    public Odometry(Drivebase drive, Limelight3A limelight) {
        this.drive = drive;
        this.limelight = limelight;
    }

    /** Initialize odometry to a known pose (robot center). */
    public void setPose(double x, double y, double headingDeg) {
        pose.x = x;
        pose.y = y;
        pose.headingDeg = headingDeg;
        double[] cur = drive.getWheelPositionsInches();
        System.arraycopy(cur, 0, lastWheelInches, 0, lastWheelInches.length);
        initialized = true;
    }

    public FieldPose getPose() { return pose.copy(); }

    /** Returns the last accepted Limelight XY pose (field frame) if available. */
    public FieldPose getLastVisionPose() { return lastVisionPose == null ? null : lastVisionPose.copy(); }

    public FieldPose getWheelPose() { return wheelPose.copy(); }

    public CameraDebug getCameraDebug() { return cameraDebug.copy(); }

    /**
     * Update pose using wheel deltas + IMU heading. Optionally blend Limelight
     * XY corrections (heading remains IMU-only).
     */
    public FieldPose update() {
        double heading = normalizeHeading(drive.heading());

        double[] wheels = drive.getWheelPositionsInches();
        long nowNs = System.nanoTime();
        double dtSec = (lastUpdateNanos > 0) ? (nowNs - lastUpdateNanos) / 1e9 : 0.0;
        lastUpdateNanos = nowNs;

        if (!initialized) {
            System.arraycopy(wheels, 0, lastWheelInches, 0, wheels.length);
            pose.headingDeg = heading;
            initialized = true;
            return pose.copy();
        }

        double fl = wheels[0] - lastWheelInches[0];
        double fr = wheels[1] - lastWheelInches[1];
        double bl = wheels[2] - lastWheelInches[2];
        double br = wheels[3] - lastWheelInches[3];
        System.arraycopy(wheels, 0, lastWheelInches, 0, wheels.length);

        double forward = -(fl + fr + bl + br) / 4.0; // +Y toward targets (invert to match field frame)
        double strafeRaw = -(fl - fr - bl + br) / 4.0; // +X right (invert to match field frame)
        double strafe = strafeRaw / Drivebase.STRAFE_CORRECTION;

        double hRad = toRadians(heading);
        double dx = strafe * cos(hRad) + forward * sin(hRad);
        double dy = forward * cos(hRad) - strafe * sin(hRad);

        double speedInPerS = (dtSec > 1e-6) ? hypot(dx, dy) / dtSec : 0.0;
        double turnRateDegPerS = (dtSec > 1e-6) ? abs(normHeading(heading - pose.headingDeg)) / dtSec : 0.0;

        FieldPose integrated = new FieldPose(
                pose.x + dx,
                pose.y + dy,
                heading
        );
        wheelPose = integrated.copy();

        FieldPose blended = applyCameraFusion(integrated, speedInPerS, turnRateDegPerS);
        pose = lowPass(pose, blended, OdometryConfig.POSE_FILTER_STRENGTH);
        return pose.copy();
    }

    private FieldPose applyCameraFusion(FieldPose base, double speedInPerS, double turnRateDegPerS) {
        cameraDebug = new CameraDebug();
        if (!OdometryConfig.USE_CAMERA_FUSION || !VisionConfig.CameraFusion.ENABLE_POSE_FUSION || limelight == null) {
            cameraPoseGate.clear();
            return base;
        }

        if (speedInPerS > VisionConfig.CameraFusion.MAX_SPEED_IN_PER_S
                || turnRateDegPerS > VisionConfig.CameraFusion.MAX_TURN_RATE_DEG_PER_S) {
            cameraDebug.rejectReason = "motionGate";
            return base;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            cameraPoseGate.clear();
            cameraDebug.rejectReason = "noResult";
            return base;
        }

        Pose3D pose3D = selectPose(result);
        if (pose3D == null || pose3D.getPosition() == null) {
            cameraPoseGate.clear();
            cameraDebug.rejectReason = "noPose";
            return base;
        }

        long nowMs = System.currentTimeMillis();
        Long timestampMs = readTimestampMs(result);
        long ageMs = (timestampMs != null) ? nowMs - timestampMs : 0;
        if (timestampMs != null && ageMs > VisionConfig.CameraFusion.CAMERA_POSE_MAX_AGE_MS) {
            cameraPoseGate.clear();
            cameraDebug.rejectReason = "stale";
            cameraDebug.ageMs = ageMs;
            return base;
        }

        List<Integer> visibleIds = extractVisibleIds(result);
        boolean goalVisible = visibleIds.contains(VisionAprilTag.TAG_BLUE_GOAL) || visibleIds.contains(VisionAprilTag.TAG_RED_GOAL);
        boolean obeliskOnly = !visibleIds.isEmpty() && visibleIds.stream().allMatch(OBELISK_TAGS::contains);
        if (obeliskOnly || !goalVisible) {
            cameraPoseGate.clear();
            cameraDebug.goalVisible = goalVisible;
            cameraDebug.obeliskOnly = obeliskOnly;
            cameraDebug.rejectReason = obeliskOnly ? "obeliskOnly" : "goalMissing";
            cameraDebug.tagIds = visibleIds;
            return base;
        }

        FieldPose camPose = mapCameraPose(pose3D);
        lastVisionPose = new FieldPose(camPose.x, camPose.y, base.headingDeg);
        double yawDeg = (pose3D.getOrientation() != null)
                ? pose3D.getOrientation().getYaw(AngleUnit.DEGREES)
                : 0.0;
        cameraDebug.rawMeters = new double[]{pose3D.getPosition().x, pose3D.getPosition().y, yawDeg};
        cameraDebug.mappedPose = camPose.copy();
        cameraDebug.goalVisible = goalVisible;
        cameraDebug.obeliskOnly = obeliskOnly;
        cameraDebug.tagIds = visibleIds;
        cameraDebug.ageMs = ageMs;

        CameraPoseGate.GateResult gate = cameraPoseGate.offer(camPose, timestampMs != null ? timestampMs : nowMs, nowMs);
        cameraDebug.windowCount = gate.windowSize;
        cameraDebug.goodCount = gate.goodCount;
        cameraDebug.stddevPosIn = gate.stddevPosIn;
        cameraDebug.stddevHeadingDeg = gate.stddevHeadingDeg;
        if (!gate.stable) {
            cameraDebug.rejectReason = gate.reason;
            return base;
        }

        double ex = camPose.x - base.x;
        double ey = camPose.y - base.y;
        double[] step = clampMagnitude(ex, ey, VisionConfig.CameraFusion.MAX_CORRECTION_STEP_IN);
        FieldPose corrected = new FieldPose(
                base.x + step[0] * VisionConfig.CameraFusion.FUSION_ALPHA,
                base.y + step[1] * VisionConfig.CameraFusion.FUSION_ALPHA,
                base.headingDeg
        );
        lastAcceptedVisionMs = nowMs;
        cameraDebug.accepted = true;
        cameraDebug.rejectReason = null;
        return corrected;
    }

    private Pose3D selectPose(LLResult result) {
        if (result == null) return null;
        Pose3D pose = null;
        if (VisionConfig.CameraFusion.PREFER_MEGA_TAG_2) {
            pose = result.getBotpose_MT2();
        }
        if (pose == null) {
            pose = result.getBotpose();
        }
        return pose;
    }

    private FieldPose mapCameraPose(Pose3D pose3D) {
        double xIn = pose3D.getPosition().x * M_TO_IN;
        double yIn = pose3D.getPosition().y * M_TO_IN;

        double tx = VisionConfig.CameraFusion.AXIS_SWAP_XY ? yIn : xIn;
        double ty = VisionConfig.CameraFusion.AXIS_SWAP_XY ? xIn : yIn;

        tx *= VisionConfig.CameraFusion.X_SIGN;
        ty *= VisionConfig.CameraFusion.Y_SIGN;

        tx += VisionConfig.CameraFusion.X_OFFSET_IN;
        ty += VisionConfig.CameraFusion.Y_OFFSET_IN;

        double heading = 0.0;
        if (pose3D.getOrientation() != null) {
            heading = pose3D.getOrientation().getYaw(AngleUnit.DEGREES);
        }
        heading = normHeading(heading);
        return new FieldPose(tx, ty, heading);
    }

    private List<Integer> extractVisibleIds(LLResult result) {
        List<Integer> ids = new ArrayList<>();
        List<?> fiducialResults = readList(result, "getFiducialResults");
        if (fiducialResults != null) {
            for (Object entry : fiducialResults) {
                Integer id = readSingleId(entry, "getFiducialId", "getTid", "getTargetId");
                if (id != null) ids.add(id);
            }
        }
        if (ids.isEmpty()) {
            int[] array = readIntArray(result, "getFiducialResults", "getFiducialIds", "getTargetIds", "getTidList");
            if (array != null) {
                for (int id : array) ids.add(id);
            }
        }
        if (ids.isEmpty()) {
            double[] array = readDoubleArray(result, "getFiducialResults", "getFiducialIds", "getTargetIds", "getTidList");
            if (array != null) {
                for (double id : array) ids.add((int) Math.round(id));
            }
        }
        if (ids.isEmpty()) {
            Integer id = readSingleId(result, "getFiducialId", "getTid", "getTargetId");
            if (id != null) ids.add(id);
        }
        return ids;
    }

    private List<?> readList(Object target, String... methods) {
        for (String m : methods) {
            try {
                Object value = target.getClass().getMethod(m).invoke(target);
                if (value instanceof List<?>) return (List<?>) value;
            } catch (Throwable ignored) {}
        }
        return null;
    }

    private Integer readSingleId(Object target, String... methods) {
        for (String m : methods) {
            try {
                Object value = target.getClass().getMethod(m).invoke(target);
                if (value instanceof Number) return ((Number) value).intValue();
            } catch (Throwable ignored) {}
        }
        return null;
    }

    private int[] readIntArray(Object target, String... methods) {
        for (String m : methods) {
            try {
                Object value = target.getClass().getMethod(m).invoke(target);
                if (value instanceof int[]) return (int[]) value;
            } catch (Throwable ignored) {}
        }
        return null;
    }

    private double[] readDoubleArray(Object target, String... methods) {
        for (String m : methods) {
            try {
                Object value = target.getClass().getMethod(m).invoke(target);
                if (value instanceof double[]) return (double[]) value;
            } catch (Throwable ignored) {}
        }
        return null;
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

    private FieldPose lowPass(FieldPose current, FieldPose target, double alpha) {
        double a = clampValue(alpha, 0.0, 1.0);
        return new FieldPose(
                current.x + (target.x - current.x) * a,
                current.y + (target.y - current.y) * a,
                normHeading(current.headingDeg + (target.headingDeg - current.headingDeg) * a)
        );
    }

    /**
     * Normalize IMU heading into the shared odometry frame where 0° faces the
     * target wall (+Y) and positive angles turn counter-clockwise toward +X.
     */
    private double normalizeHeading(double rawHeadingDeg) {
        double shifted = rawHeadingDeg + OdometryConfig.IMU_HEADING_OFFSET_DEG;
        double fieldFrame = shifted - 90.0; // align IMU basis (0° = +X) to odometry basis (0° = +Y)
        return normHeading(fieldFrame);
    }

    private static double normHeading(double h) {
        double v = h % 360.0;
        if (v < -180) v += 360.0;
        if (v > 180) v -= 360.0;
        return v;
    }

    private double[] clampMagnitude(double x, double y, double maxMag) {
        double mag = hypot(x, y);
        if (mag <= maxMag || mag < 1e-6) return new double[]{x, y};
        double scale = maxMag / mag;
        return new double[]{x * scale, y * scale};
    }

    private double clampValue(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    public static class CameraDebug {
        public double[] rawMeters = null;
        public FieldPose mappedPose = null;
        public boolean accepted = false;
        public boolean goalVisible = false;
        public boolean obeliskOnly = false;
        public String rejectReason = "notRun";
        public int windowCount = 0;
        public int goodCount = 0;
        public double stddevPosIn = 0.0;
        public double stddevHeadingDeg = 0.0;
        public long ageMs = 0;
        public List<Integer> tagIds = Collections.emptyList();

        public CameraDebug copy() {
            CameraDebug c = new CameraDebug();
            c.rawMeters = rawMeters;
            c.mappedPose = (mappedPose == null) ? null : mappedPose.copy();
            c.accepted = accepted;
            c.goalVisible = goalVisible;
            c.obeliskOnly = obeliskOnly;
            c.rejectReason = rejectReason;
            c.windowCount = windowCount;
            c.goodCount = goodCount;
            c.stddevPosIn = stddevPosIn;
            c.stddevHeadingDeg = stddevHeadingDeg;
            c.ageMs = ageMs;
            c.tagIds = new ArrayList<>(tagIds);
            return c;
        }
    }

    private static class CameraPoseGate {
        private final Deque<StampedPose> window = new ArrayDeque<>();

        public void clear() { window.clear(); }

        public GateResult offer(FieldPose pose, long poseMs, long nowMs) {
            window.addLast(new StampedPose(pose.copy(), poseMs));
            while (window.size() > VisionConfig.CameraFusion.CAMERA_POSE_WINDOW_FRAMES ||
                    (!window.isEmpty() && nowMs - window.getFirst().timestampMs > VisionConfig.CameraFusion.CAMERA_POSE_MAX_AGE_MS)) {
                window.removeFirst();
            }

            double stdPos = 0.0;
            double stdHead = 0.0;
            if (!window.isEmpty()) {
                double meanX = 0, meanY = 0;
                double meanSin = 0, meanCos = 0;
                for (StampedPose sp : window) {
                    meanX += sp.pose.x;
                    meanY += sp.pose.y;
                    double rad = toRadians(sp.pose.headingDeg);
                    meanSin += Math.sin(rad);
                    meanCos += Math.cos(rad);
                }
                int n = window.size();
                meanX /= n; meanY /= n;
                meanSin /= n; meanCos /= n;
                double meanHeading = Math.atan2(meanSin, meanCos);

                double varPos = 0.0;
                double varHead = 0.0;
                for (StampedPose sp : window) {
                    varPos += pow(sp.pose.x - meanX, 2) + pow(sp.pose.y - meanY, 2);
                    double d = normHeading(sp.pose.headingDeg - toDegrees(meanHeading));
                    varHead += d * d;
                }
                stdPos = Math.sqrt(varPos / n);
                stdHead = Math.sqrt(varHead / n);
            }

            boolean stable = window.size() >= VisionConfig.CameraFusion.CAMERA_POSE_MIN_GOOD_FRAMES
                    && stdPos <= VisionConfig.CameraFusion.CAMERA_POSE_MAX_STDDEV_IN
                    && stdHead <= VisionConfig.CameraFusion.CAMERA_POSE_MAX_STDDEV_DEG;
            String reason = stable ? null : "unstable";

            return new GateResult(window.size(), window.size(), stdPos, stdHead, stable, reason);
        }

        public static class GateResult {
            public final int windowSize;
            public final int goodCount;
            public final double stddevPosIn;
            public final double stddevHeadingDeg;
            public final boolean stable;
            public final String reason;

            public GateResult(int windowSize, int goodCount, double stddevPosIn, double stddevHeadingDeg, boolean stable, String reason) {
                this.windowSize = windowSize;
                this.goodCount = goodCount;
                this.stddevPosIn = stddevPosIn;
                this.stddevHeadingDeg = stddevHeadingDeg;
                this.stable = stable;
                this.reason = reason;
            }
        }

        private static class StampedPose {
            final FieldPose pose;
            final long timestampMs;
            StampedPose(FieldPose pose, long timestampMs) { this.pose = pose; this.timestampMs = timestampMs; }
        }
    }
}
