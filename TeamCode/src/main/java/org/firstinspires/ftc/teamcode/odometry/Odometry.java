package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.OdometryConfig;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.drive.Drivebase;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Locale;

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
 *   - Limelight corrections blend position only (X/Y); yaw is always IMU-only.
 *
 * CHANGES (2025-12-13): Documented the FieldPose convention (inches, +X right,
 *                        +Y toward targets, +CCW heading), added a stability
 *                        gate for Limelight camera fusion (windowed stddev +
 *                        age) with an alliance-safe obelisk exclusion, and
 *                        centralized camera pose mapping to the shared frame
 *                        while exposing fusion debug telemetry.
 * CHANGES (2025-12-14): Restored SDK compatibility by reflection-reading
 *                        fiducial IDs when LLFiducialResult is absent and
 *                        tightened static helpers used by the camera gate.
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
    private FieldPose wheelOnlyPose = new FieldPose();

    private long lastUpdateNanos = -1L;
    private long lastAcceptedVisionMs = -1L;
    private FieldPose lastVisionPose = null;

    private final CameraPoseGate cameraPoseGate = new CameraPoseGate();
    private CameraFusionTelemetry fusionTelemetry = new CameraFusionTelemetry();

    private static final double M_TO_IN = 39.37007874;

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

    /** Returns the most recent wheel-only pose (without camera fusion). */
    public FieldPose getWheelOnlyPose() { return wheelOnlyPose.copy(); }

    /** Returns the last accepted Limelight XY pose (field frame) if available. */
    public FieldPose getLastVisionPose() { return lastVisionPose == null ? null : lastVisionPose.copy(); }

    /** Returns the latest camera fusion debug snapshot. */
    public CameraFusionTelemetry getFusionTelemetry() { return fusionTelemetry.copy(); }

    /**
     * Update pose using wheel deltas + IMU heading. Optionally blend Limelight
     * XY corrections (heading remains IMU-only).
     *
     * FieldPose convention:
     *   - Units: inches for X/Y.
     *   - Axes: +X to the right when facing the targets, +Y toward the targets.
     *   - Heading: 0° faces +Y; positive angles turn counter-clockwise toward +X.
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

        wheelOnlyPose = integrated.copy();

        FieldPose blended = applyLimelightFusion(integrated, speedInPerS, turnRateDegPerS);
        pose = lowPass(pose, blended, OdometryConfig.POSE_FILTER_STRENGTH);
        return pose.copy();
    }

    private FieldPose applyLimelightFusion(FieldPose base, double speedInPerS, double turnRateDegPerS) {
        fusionTelemetry = new CameraFusionTelemetry();
        fusionTelemetry.wheelPose = base.copy();

        if (!OdometryConfig.USE_CAMERA_FUSION || !VisionConfig.LimelightFusion.ENABLE_POSE_FUSION || limelight == null) {
            fusionTelemetry.rejectReason = "fusion disabled";
            return base;
        }

        if (speedInPerS > VisionConfig.LimelightFusion.MAX_SPEED_IN_PER_S
                || turnRateDegPerS > VisionConfig.LimelightFusion.MAX_TURN_RATE_DEG_PER_S) {
            fusionTelemetry.rejectReason = String.format(Locale.US, "motion gate v=%.1f t=%.1f", speedInPerS, turnRateDegPerS);
            return base;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            fusionTelemetry.rejectReason = "no result";
            return base;
        }

        List<Integer> visibleIds = getVisibleIds(result);
        fusionTelemetry.visibleIds = visibleIds;
        boolean hasGoal = visibleIds.contains(VisionConfig.GOAL_TAG_BLUE) || visibleIds.contains(VisionConfig.GOAL_TAG_RED);
        boolean obeliskOnly = !visibleIds.isEmpty() && !hasGoal && allObelisk(visibleIds);
        fusionTelemetry.goalVisible = hasGoal;
        fusionTelemetry.obeliskOnly = obeliskOnly;
        if (obeliskOnly || (!hasGoal && !visibleIds.isEmpty())) {
            fusionTelemetry.rejectReason = obeliskOnly ? "obelisk-only" : "no goal id";
            return base;
        }

        CameraPose cameraPose = mapCameraPose(result);
        if (cameraPose == null) {
            fusionTelemetry.rejectReason = "no pose";
            return base;
        }
        fusionTelemetry.cameraRawMeters = cameraPose.rawMeters;
        fusionTelemetry.cameraPoseInches = cameraPose.fieldPose.copy();

        long nowMs = System.currentTimeMillis();
        long ageMs = nowMs - cameraPose.timestampMs;
        fusionTelemetry.sampleAgeMs = ageMs;
        if (ageMs > OdometryConfig.CAMERA_POSE_MAX_AGE_MS) {
            fusionTelemetry.rejectReason = "pose too old";
            return base;
        }

        cameraPoseGate.add(cameraPose.fieldPose, cameraPose.timestampMs);
        CameraPoseGate.GateResult gate = cameraPoseGate.evaluate(nowMs);
        fusionTelemetry.gateWindowCount = gate.windowCount;
        fusionTelemetry.gateGoodFrames = gate.goodFrames;
        fusionTelemetry.gateStddevIn = gate.stddevIn;
        fusionTelemetry.gateStddevDeg = gate.stddevDeg;
        fusionTelemetry.gateAgeMs = gate.ageMs;
        fusionTelemetry.gateStable = gate.stable;
        fusionTelemetry.gateRejectReason = gate.rejectReason;
        if (!gate.stable) {
            fusionTelemetry.rejectReason = gate.rejectReason;
            return base;
        }

        FieldPose fused = new FieldPose(
                base.x + (gate.meanPose.x - base.x) * OdometryConfig.CAMERA_FUSE_ALPHA,
                base.y + (gate.meanPose.y - base.y) * OdometryConfig.CAMERA_FUSE_ALPHA,
                base.headingDeg
        );
        lastVisionPose = gate.meanPose.copy();
        lastAcceptedVisionMs = nowMs;
        fusionTelemetry.accepted = true;
        fusionTelemetry.rejectReason = "";
        return fused;
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

    private boolean allObelisk(List<Integer> ids) {
        for (int id : ids) {
            if (id < 21 || id > 23) {
                return false;
            }
        }
        return !ids.isEmpty();
    }

    private List<Integer> getVisibleIds(LLResult result) {
        List<Integer> ids = new ArrayList<>();
        if (result == null) return ids;
        try {
            Object rawList = result.getClass().getMethod("getFiducialResults").invoke(result);
            if (rawList instanceof List<?>) {
                for (Object f : (List<?>) rawList) {
                    try {
                        Object idObj = f.getClass().getMethod("getFiducialId").invoke(f);
                        if (idObj instanceof Number) {
                            ids.add(((Number) idObj).intValue());
                        }
                    } catch (Throwable ignoredInner) { }
                }
            }
        } catch (Throwable ignored) { }
        return ids;
    }

    private CameraPose mapCameraPose(LLResult result) {
        Pose3D pose3D = selectPose(result);
        if (pose3D == null || pose3D.getPosition() == null) return null;

        double rawX = pose3D.getPosition().x;
        double rawY = pose3D.getPosition().y;
        double xIn = rawX * M_TO_IN;
        double yIn = rawY * M_TO_IN;

        double tx = VisionConfig.LimelightFusion.AXIS_SWAP_XY ? yIn : xIn;
        double ty = VisionConfig.LimelightFusion.AXIS_SWAP_XY ? xIn : yIn;

        tx *= VisionConfig.LimelightFusion.X_SIGN;
        ty *= VisionConfig.LimelightFusion.Y_SIGN;

        tx += VisionConfig.LimelightFusion.X_OFFSET_IN;
        ty += VisionConfig.LimelightFusion.Y_OFFSET_IN;

        double yawDeg = 0.0;
        try {
            yawDeg = pose3D.getOrientation().getYaw(AngleUnit.DEGREES);
        } catch (Throwable ignored) { }
        double fieldHeading = normHeading(yawDeg - 90.0);

        Long timestampMs = readTimestampMs(result);
        long ts = (timestampMs != null) ? timestampMs : System.currentTimeMillis();

        return new CameraPose(ts,
                new FieldPose(tx, ty, fieldHeading),
                new double[]{rawX, rawY, yawDeg});
    }

    private Pose3D selectPose(LLResult result) {
        if (result == null) return null;
        Pose3D pose = null;
        if (VisionConfig.LimelightFusion.PREFER_MEGA_TAG_2) {
            pose = result.getBotpose_MT2();
        }
        if (pose == null) {
            pose = result.getBotpose();
        }
        return pose;
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

    /** Rolling stability gate for camera poses. */
    private static class CameraPoseGate {
        private final Deque<StampedPose> window = new ArrayDeque<>();

        void add(FieldPose pose, long timestampMs) {
            window.addLast(new StampedPose(pose.copy(), timestampMs));
            while (window.size() > OdometryConfig.CAMERA_POSE_WINDOW_FRAMES) {
                window.removeFirst();
            }
        }

        GateResult evaluate(long nowMs) {
            while (!window.isEmpty() && nowMs - window.peekFirst().timestampMs > OdometryConfig.CAMERA_POSE_MAX_AGE_MS) {
                window.removeFirst();
            }

            GateResult result = new GateResult();
            result.windowCount = window.size();
            if (window.isEmpty()) {
                result.rejectReason = "no samples";
                return result;
            }

            double meanX = 0, meanY = 0, meanH = 0;
            for (StampedPose s : window) {
                meanX += s.pose.x;
                meanY += s.pose.y;
                meanH += s.pose.headingDeg;
            }
            meanX /= window.size();
            meanY /= window.size();
            meanH /= window.size();

            double varX = 0, varY = 0, varH = 0;
            long newestTs = 0;
            for (StampedPose s : window) {
                varX += pow(s.pose.x - meanX, 2);
                varY += pow(s.pose.y - meanY, 2);
                varH += pow(normHeading(s.pose.headingDeg - meanH), 2);
                newestTs = Math.max(newestTs, s.timestampMs);
            }
            varX /= window.size();
            varY /= window.size();
            varH /= window.size();

            result.stddevIn = sqrt(max(varX, varY));
            result.stddevDeg = sqrt(varH);
            result.ageMs = nowMs - newestTs;
            result.meanPose = new FieldPose(meanX, meanY, meanH);

            if (window.size() < OdometryConfig.CAMERA_POSE_MIN_GOOD_FRAMES) {
                result.rejectReason = "min frames";
                return result;
            }
            if (result.stddevIn > OdometryConfig.CAMERA_POSE_MAX_STDDEV_IN) {
                result.rejectReason = "pos stddev";
                return result;
            }
            if (result.stddevDeg > OdometryConfig.CAMERA_POSE_MAX_STDDEV_DEG) {
                result.rejectReason = "heading stddev";
                return result;
            }

            result.stable = true;
            result.rejectReason = "";
            result.goodFrames = window.size();
            return result;
        }

        private static class StampedPose {
            final FieldPose pose;
            final long timestampMs;

            StampedPose(FieldPose pose, long timestampMs) {
                this.pose = pose;
                this.timestampMs = timestampMs;
            }
        }

        static class GateResult {
            FieldPose meanPose = new FieldPose();
            boolean stable = false;
            int windowCount = 0;
            int goodFrames = 0;
            double stddevIn = 0.0;
            double stddevDeg = 0.0;
            long ageMs = 0L;
            String rejectReason = "";
        }
    }

    /** Raw camera pose with timestamp and field-frame mapping. */
    private static class CameraPose {
        final long timestampMs;
        final FieldPose fieldPose;
        final double[] rawMeters;

        CameraPose(long timestampMs, FieldPose fieldPose, double[] rawMeters) {
            this.timestampMs = timestampMs;
            this.fieldPose = fieldPose;
            this.rawMeters = rawMeters;
        }
    }

    /** Telemetry snapshot for odometry + camera fusion. */
    public static class CameraFusionTelemetry {
        public FieldPose wheelPose = new FieldPose();
        public FieldPose cameraPoseInches = null;
        public double[] cameraRawMeters = null;
        public List<Integer> visibleIds = new ArrayList<>();
        public int gateWindowCount = 0;
        public int gateGoodFrames = 0;
        public double gateStddevIn = 0.0;
        public double gateStddevDeg = 0.0;
        public long gateAgeMs = 0L;
        public boolean gateStable = false;
        public String gateRejectReason = "";
        public boolean accepted = false;
        public boolean goalVisible = false;
        public boolean obeliskOnly = false;
        public long sampleAgeMs = -1L;
        public String rejectReason = "";

        public CameraFusionTelemetry copy() {
            CameraFusionTelemetry t = new CameraFusionTelemetry();
            t.wheelPose = wheelPose.copy();
            t.cameraPoseInches = (cameraPoseInches == null) ? null : cameraPoseInches.copy();
            t.cameraRawMeters = cameraRawMeters == null ? null : cameraRawMeters.clone();
            t.visibleIds = new ArrayList<>(visibleIds);
            t.gateWindowCount = gateWindowCount;
            t.gateGoodFrames = gateGoodFrames;
            t.gateStddevIn = gateStddevIn;
            t.gateStddevDeg = gateStddevDeg;
            t.gateAgeMs = gateAgeMs;
            t.gateStable = gateStable;
            t.gateRejectReason = gateRejectReason;
            t.accepted = accepted;
            t.goalVisible = goalVisible;
            t.obeliskOnly = obeliskOnly;
            t.sampleAgeMs = sampleAgeMs;
            t.rejectReason = rejectReason;
            return t;
        }
    }
}
