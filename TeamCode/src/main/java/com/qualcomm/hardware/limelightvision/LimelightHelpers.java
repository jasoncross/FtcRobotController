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
}
