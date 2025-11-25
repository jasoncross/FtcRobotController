package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.teamcode.config.OdometryConfig;
import org.firstinspires.ftc.teamcode.drive.Drivebase;
import org.firstinspires.ftc.teamcode.vision.VisionAprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import static java.lang.Math.*;

/*
 * FILE: Odometry.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/odometry/
 *
 * PURPOSE
 *   - Fuse drivetrain encoder deltas, IMU heading, and optional AprilTag goal
 *     detections into a stable field pose anchored at the robot center.
 *   - Publish a consistent coordinate frame for TeleOp, Auto, and dashboard
 *     drawing utilities without duplicating geometry math across callers.
 *
 * NOTES
 *   - Heading comes from Drivebase.heading() plus the configurable IMU offset.
 *   - AprilTag corrections are blended gently (VISION_WEIGHT) and limited to a
 *     maximum correction step to avoid teleporting the pose when detections
 *     spike.
 *
 * CHANGES (2025-11-25): Added goal-tag pose fusion that leverages config tag poses,
 *                        camera offsets, and pitch/yaw to compute the robot center
 *                        from detections, enabling Auto→TeleOp pose carryover.
 */
public class Odometry {

    private final Drivebase drive;
    private final VisionAprilTag vision;

    private final double[] lastWheelInches = new double[4];
    private boolean initialized = false;
    private FieldPose pose = new FieldPose();

    private static final double M_TO_IN = 39.37007874;

    private static final class TagPose {
        final double x;
        final double y;
        final double z;
        final double yawDeg;

        TagPose(double x, double y, double z, double yawDeg) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.yawDeg = yawDeg;
        }
    }

    private TagPose goalTagPoseFor(int id) {
        if (id == VisionAprilTag.TAG_RED_GOAL) {
            return new TagPose(OdometryConfig.TAG_RED_GOAL_X, OdometryConfig.TAG_RED_GOAL_Y, OdometryConfig.TAG_RED_GOAL_Z, OdometryConfig.TAG_RED_GOAL_YAW_DEG);
        } else if (id == VisionAprilTag.TAG_BLUE_GOAL) {
            return new TagPose(OdometryConfig.TAG_BLUE_GOAL_X, OdometryConfig.TAG_BLUE_GOAL_Y, OdometryConfig.TAG_BLUE_GOAL_Z, OdometryConfig.TAG_BLUE_GOAL_YAW_DEG);
        }
        return null;
    }

    public Odometry(Drivebase drive, VisionAprilTag vision) {
        this.drive = drive;
        this.vision = vision;
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

    /**
     * Update pose using wheel deltas + IMU heading. Optionally supply a goal
     * AprilTag detection for vision corrections.
     */
    public FieldPose update(AprilTagDetection detection) {
        double heading = normHeading(drive.heading() + OdometryConfig.IMU_HEADING_OFFSET_DEG);

        double[] wheels = drive.getWheelPositionsInches();
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

        double forward = (fl + fr + bl + br) / 4.0; // +Y toward targets
        double strafeRaw = (fl - fr - bl + br) / 4.0; // +X right
        double strafe = strafeRaw / Drivebase.STRAFE_CORRECTION;

        double hRad = toRadians(heading);
        double dx = strafe * cos(hRad) + forward * sin(hRad);
        double dy = forward * cos(hRad) - strafe * sin(hRad);

        FieldPose integrated = new FieldPose(
                pose.x + dx,
                pose.y + dy,
                heading
        );

        FieldPose blended = applyVision(integrated, detection, heading);
        pose = lowPass(pose, blended, OdometryConfig.POSE_FILTER_STRENGTH);
        return pose.copy();
    }

    public FieldPose computeVisionPose(AprilTagDetection det, double headingDeg) {
        if (vision == null || det == null || det.ftcPose == null) return null;
        TagPose tag = goalTagPoseFor(det.id);
        if (tag == null) return null;

        double rangeMeters = det.ftcPose.range;
        double bearingDeg = det.ftcPose.bearing;
        double elevationDeg = det.ftcPose.elevation;
        if (Double.isNaN(rangeMeters) || Double.isNaN(bearingDeg)) return null;

        double scaledRangeIn = vision.getScaledRange(det) * M_TO_IN;
        double elevationRad = toRadians(elevationDeg);
        double horizontalRange = scaledRangeIn;
        double verticalDelta = tag.z - OdometryConfig.CAMERA_OFFSET_Z;
        double tanElev = tan(elevationRad);
        if (!Double.isNaN(tanElev) && abs(tanElev) > 1e-3) {
            double derivedHorizontal = abs(verticalDelta / tanElev);
            if (!Double.isNaN(derivedHorizontal) && derivedHorizontal > 0.0) {
                horizontalRange = derivedHorizontal;
            }
        } else if (!Double.isNaN(elevationRad)) {
            horizontalRange = scaledRangeIn * cos(elevationRad);
        }

        double bearingRad = toRadians(bearingDeg + OdometryConfig.CAMERA_YAW_DEG);
        double relXCam = horizontalRange * sin(bearingRad); // +X camera right
        double relYCam = horizontalRange * cos(bearingRad); // +Y camera forward

        // Vector from tag to camera in the tag frame, then rotate by tag yaw into field coords.
        double tagToCamX = -relXCam;
        double tagToCamY = -relYCam;
        double tagYawRad = toRadians(tag.yawDeg);
        double fieldCamOffsetX = tagToCamX * cos(tagYawRad) - tagToCamY * sin(tagYawRad);
        double fieldCamOffsetY = tagToCamX * sin(tagYawRad) + tagToCamY * cos(tagYawRad);

        double camX = tag.x + fieldCamOffsetX;
        double camY = tag.y + fieldCamOffsetY;

        double headingRad = toRadians(normHeading(headingDeg));
        double camHeading = headingRad + toRadians(OdometryConfig.CAMERA_YAW_DEG);
        double offsetXField = OdometryConfig.CAMERA_OFFSET_X * cos(camHeading) + OdometryConfig.CAMERA_OFFSET_Y * sin(camHeading);
        double offsetYField = OdometryConfig.CAMERA_OFFSET_Y * cos(camHeading) - OdometryConfig.CAMERA_OFFSET_X * sin(camHeading);

        return new FieldPose(
                camX - offsetXField,
                camY - offsetYField,
                headingDeg
        );
    }

    private FieldPose applyVision(FieldPose base, AprilTagDetection det, double headingDeg) {
        FieldPose visionPose = computeVisionPose(det, headingDeg);
        if (visionPose == null) return base;

        double corrX = clampValue(visionPose.x - base.x, -OdometryConfig.MAX_VISION_CORRECTION_DISTANCE, OdometryConfig.MAX_VISION_CORRECTION_DISTANCE);
        double corrY = clampValue(visionPose.y - base.y, -OdometryConfig.MAX_VISION_CORRECTION_DISTANCE, OdometryConfig.MAX_VISION_CORRECTION_DISTANCE);

        return new FieldPose(
                base.x + corrX * OdometryConfig.VISION_WEIGHT,
                base.y + corrY * OdometryConfig.VISION_WEIGHT,
                base.headingDeg
        );
    }

    private FieldPose lowPass(FieldPose current, FieldPose target, double alpha) {
        double a = clampValue(alpha, 0.0, 1.0);
        return new FieldPose(
                current.x + (target.x - current.x) * a,
                current.y + (target.y - current.y) * a,
                normHeading(current.headingDeg + (target.headingDeg - current.headingDeg) * a)
        );
    }

    private double normHeading(double h) {
        double v = h % 360.0;
        if (v < -180) v += 360.0;
        if (v > 180) v -= 360.0;
        return v;
    }

    private double clampValue(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
