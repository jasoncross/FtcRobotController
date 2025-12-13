package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.OdometryConfig;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.drive.Drivebase;

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

    private long lastUpdateNanos = -1L;
    private long lastAcceptedVisionMs = -1L;
    private int validVisionStreak = 0;
    private FieldPose lastVisionPose = null;

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

    /** Returns the last accepted Limelight XY pose (field frame) if available. */
    public FieldPose getLastVisionPose() { return lastVisionPose == null ? null : lastVisionPose.copy(); }

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

        FieldPose blended = applyLimelightFusion(integrated, speedInPerS, turnRateDegPerS);
        pose = lowPass(pose, blended, OdometryConfig.POSE_FILTER_STRENGTH);
        return pose.copy();
    }

    private FieldPose applyLimelightFusion(FieldPose base, double speedInPerS, double turnRateDegPerS) {
        if (!VisionConfig.LimelightFusion.ENABLE_POSE_FUSION || limelight == null) {
            validVisionStreak = 0;
            return base;
        }

        if (speedInPerS > VisionConfig.LimelightFusion.MAX_SPEED_IN_PER_S
                || turnRateDegPerS > VisionConfig.LimelightFusion.MAX_TURN_RATE_DEG_PER_S) {
            return base;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            validVisionStreak = 0;
            return base;
        }

        Pose3D pose3D = selectPose(result);
        if (pose3D == null || pose3D.getPosition() == null) {
            validVisionStreak = 0;
            return base;
        }

        long nowMs = System.currentTimeMillis();
        Long timestampMs = readTimestampMs(result);
        if (timestampMs != null) {
            long ageMs = nowMs - timestampMs;
            if (ageMs > VisionConfig.LimelightFusion.MAX_AGE_MS) {
                validVisionStreak = 0;
                return base;
            }
        }

        validVisionStreak++;
        if (validVisionStreak < VisionConfig.LimelightFusion.MIN_VALID_FRAMES) {
            return base;
        }

        double[] xy = transformVisionXY(pose3D.getPosition().x, pose3D.getPosition().y);
        double vx = xy[0];
        double vy = xy[1];
        lastVisionPose = new FieldPose(vx, vy, base.headingDeg);

        long sinceLast = (lastAcceptedVisionMs < 0) ? Long.MAX_VALUE : nowMs - lastAcceptedVisionMs;
        boolean reacquire = sinceLast > VisionConfig.LimelightFusion.REACQUIRE_AFTER_MS;
        double maxJump = reacquire
                ? VisionConfig.LimelightFusion.MAX_POS_JUMP_IN_REACQUIRE
                : VisionConfig.LimelightFusion.MAX_POS_JUMP_IN_NORMAL;
        double alpha = reacquire
                ? VisionConfig.LimelightFusion.FUSION_ALPHA_REACQUIRE
                : VisionConfig.LimelightFusion.FUSION_ALPHA_NORMAL;

        double ex = vx - base.x;
        double ey = vy - base.y;
        double errorMag = hypot(ex, ey);
        if (errorMag > maxJump) {
            return base;
        }

        double[] step = clampMagnitude(ex, ey, VisionConfig.LimelightFusion.MAX_CORRECTION_STEP_IN);
        FieldPose corrected = new FieldPose(
                base.x + step[0] * alpha,
                base.y + step[1] * alpha,
                base.headingDeg
        );
        lastAcceptedVisionMs = nowMs;
        return corrected;
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

    private double[] transformVisionXY(double xMeters, double yMeters) {
        double xIn = xMeters * M_TO_IN;
        double yIn = yMeters * M_TO_IN;

        double tx = VisionConfig.LimelightFusion.AXIS_SWAP_XY ? yIn : xIn;
        double ty = VisionConfig.LimelightFusion.AXIS_SWAP_XY ? xIn : yIn;

        tx *= VisionConfig.LimelightFusion.X_SIGN;
        ty *= VisionConfig.LimelightFusion.Y_SIGN;

        tx += VisionConfig.LimelightFusion.X_OFFSET_IN;
        ty += VisionConfig.LimelightFusion.Y_OFFSET_IN;

        return new double[]{tx, ty};
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

    private double normHeading(double h) {
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
}
