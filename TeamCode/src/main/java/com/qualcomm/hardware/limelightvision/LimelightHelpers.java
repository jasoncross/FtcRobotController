package com.qualcomm.hardware.limelightvision;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/*
 * FILE: LimelightHelpers.java
 * LOCATION: TeamCode/src/main/java/com/qualcomm/hardware/limelightvision/
 *
 * PURPOSE
 *   - Provide FTC-safe helpers for Limelight MT2 yaw feeding and fiducial
 *     filtering without relying on WPILib NetworkTables.
 *   - Keep a lightweight registry of active Limelight3A devices so callers
 *     can address them by the configured NetworkTables name.
 */
public final class LimelightHelpers {
    private LimelightHelpers() {}

    private static final Map<String, Limelight3A> LIMELIGHTS = new ConcurrentHashMap<>();

    public static final class Pose2d {
        public final double x;
        public final double y;
        public final double headingRad;

        public Pose2d(double x, double y, double headingRad) {
            this.x = x;
            this.y = y;
            this.headingRad = headingRad;
        }
    }

    public static final class PoseEstimate {
        public final Pose2d pose;
        public final int tagCount;
        public final double timestampSeconds;

        public PoseEstimate(Pose2d pose, int tagCount, double timestampSeconds) {
            this.pose = pose;
            this.tagCount = tagCount;
            this.timestampSeconds = timestampSeconds;
        }
    }

    public static void registerLimelight(String name, Limelight3A limelight) {
        if (name == null || limelight == null) return;
        LIMELIGHTS.put(name, limelight);
    }

    public static boolean SetRobotOrientation(String name,
                                              double yawDeg,
                                              double yawRateDegPerSec,
                                              double pitchDeg,
                                              double pitchRateDegPerSec,
                                              double rollDeg,
                                              double rollRateDegPerSec) {
        return setRobotOrientation(name,
                yawDeg,
                yawRateDegPerSec,
                pitchDeg,
                pitchRateDegPerSec,
                rollDeg,
                rollRateDegPerSec);
    }

    public static boolean setRobotOrientation(String name,
                                              double yawDeg,
                                              double yawRateDegPerSec,
                                              double pitchDeg,
                                              double pitchRateDegPerSec,
                                              double rollDeg,
                                              double rollRateDegPerSec) {
        Limelight3A limelight = LIMELIGHTS.get(name);
        if (limelight == null) return false;
        try {
            limelight.getClass().getMethod(
                    "setRobotOrientation",
                    double.class,
                    double.class,
                    double.class,
                    double.class,
                    double.class,
                    double.class
            ).invoke(limelight,
                    yawDeg,
                    yawRateDegPerSec,
                    pitchDeg,
                    pitchRateDegPerSec,
                    rollDeg,
                    rollRateDegPerSec);
            return true;
        } catch (Throwable ignored) { }
        return false;
    }

    public static boolean SetFiducialIDFiltersOverride(String name, int[] ids) {
        return setFiducialIDFiltersOverride(name, ids);
    }

    public static boolean setFiducialIDFiltersOverride(String name, int[] ids) {
        Limelight3A limelight = LIMELIGHTS.get(name);
        if (limelight == null || ids == null) return false;
        try {
            limelight.getClass().getMethod("setFiducialIDFiltersOverride", int[].class)
                    .invoke(limelight, (Object) ids);
            return true;
        } catch (Throwable ignored) { }
        return setFiducialIDFilters(name, ids);
    }

    public static boolean setFiducialIDFilters(String name, int[] ids) {
        Limelight3A limelight = LIMELIGHTS.get(name);
        if (limelight == null || ids == null) return false;
        try {
            limelight.getClass().getMethod("setFiducialIDFilters", int[].class)
                    .invoke(limelight, (Object) ids);
            return true;
        } catch (Throwable ignored) { }
        return false;
    }

    public static Pose3D getBotposeMT2Blue(LLResult result) {
        return readPose(result,
                "getBotpose_MT2_WPIBlue",
                "getBotpose_MT2_Blue",
                "getBotpose_MT2_blue");
    }

    public static Pose3D getBotposeMT2Red(LLResult result) {
        return readPose(result,
                "getBotpose_MT2_WPIRed",
                "getBotpose_MT2_Red",
                "getBotpose_MT2_red");
    }

    private static Pose3D readPose(LLResult result, String... methods) {
        if (result == null) return null;
        for (String method : methods) {
            try {
                Object value = result.getClass().getMethod(method).invoke(result);
                if (value instanceof Pose3D) {
                    return (Pose3D) value;
                }
            } catch (Throwable ignored) { }
        }
        return null;
    }

    public static PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(String name) {
        Limelight3A limelight = LIMELIGHTS.get(name);
        if (limelight == null) return null;
        PoseEstimate estimate = readPoseEstimate(limelight, "getBotPoseEstimate_wpiBlue_MegaTag2");
        if (estimate != null) return estimate;
        return fallbackPoseEstimate(limelight.getLatestResult());
    }

    private static PoseEstimate readPoseEstimate(Limelight3A limelight, String methodName) {
        try {
            Object value = limelight.getClass().getMethod(methodName).invoke(limelight);
            return coercePoseEstimate(value);
        } catch (Throwable ignored) { }
        return null;
    }

    private static PoseEstimate coercePoseEstimate(Object value) {
        if (value == null) return null;
        if (value instanceof PoseEstimate) return (PoseEstimate) value;
        try {
            Object poseObj = readMember(value, "pose", "getPose", "getPose2d");
            Pose2d pose2d = coercePose2d(poseObj);
            if (pose2d == null) return null;
            int tagCount = readInt(value, "tagCount", "getTagCount", "getTagcount");
            double timestampSeconds = readDouble(value, "timestampSeconds", "getTimestampSeconds", "getTimestamp");
            return new PoseEstimate(pose2d, tagCount, timestampSeconds);
        } catch (Throwable ignored) { }
        return null;
    }

    private static Pose2d coercePose2d(Object poseObj) {
        if (poseObj == null) return null;
        try {
            double x = readDouble(poseObj, "x", "getX");
            double y = readDouble(poseObj, "y", "getY");
            double heading = readDouble(poseObj, "headingRad", "getHeading", "getRotation", "getRotationRadians");
            if (!Double.isFinite(heading)) {
                heading = 0.0;
            }
            return new Pose2d(x, y, heading);
        } catch (Throwable ignored) { }
        return null;
    }

    private static PoseEstimate fallbackPoseEstimate(LLResult result) {
        Pose3D pose3D = readPose(result, "getBotpose_MT2_WPIBlue");
        if (pose3D == null || pose3D.getPosition() == null) return null;
        Pose2d pose2d = new Pose2d(pose3D.getPosition().x, pose3D.getPosition().y, 0.0);
        int tagCount = countFiducials(result);
        return new PoseEstimate(pose2d, tagCount, 0.0);
    }

    private static int countFiducials(LLResult result) {
        if (result == null) return 0;
        Object list = readMember(result, "getFiducialResults");
        if (list instanceof Iterable) {
            int count = 0;
            for (Object ignored : (Iterable<?>) list) {
                count++;
            }
            return count;
        }
        int[] ids = readIntArray(result, "getFiducialIds", "getTargetIds", "getTidList");
        if (ids != null) return ids.length;
        double[] idsDouble = readDoubleArray(result, "getFiducialIds", "getTargetIds", "getTidList");
        return idsDouble != null ? idsDouble.length : 0;
    }

    private static Object readMember(Object owner, String... methodsOrFields) {
        for (String name : methodsOrFields) {
            try {
                return owner.getClass().getMethod(name).invoke(owner);
            } catch (Throwable ignored) { }
            try {
                return owner.getClass().getField(name).get(owner);
            } catch (Throwable ignored) { }
        }
        return null;
    }

    private static int readInt(Object owner, String... names) {
        Object value = readMember(owner, names);
        if (value instanceof Number) {
            return ((Number) value).intValue();
        }
        return 0;
    }

    private static double readDouble(Object owner, String... names) {
        Object value = readMember(owner, names);
        if (value instanceof Number) {
            return ((Number) value).doubleValue();
        }
        return Double.NaN;
    }

    private static int[] readIntArray(Object owner, String... methods) {
        for (String method : methods) {
            try {
                Object value = owner.getClass().getMethod(method).invoke(owner);
                if (value instanceof int[]) {
                    return (int[]) value;
                }
            } catch (Throwable ignored) { }
        }
        return null;
    }

    private static double[] readDoubleArray(Object owner, String... methods) {
        for (String method : methods) {
            try {
                Object value = owner.getClass().getMethod(method).invoke(owner);
                if (value instanceof double[]) {
                    return (double[]) value;
                }
            } catch (Throwable ignored) { }
        }
        return null;
    }
}
