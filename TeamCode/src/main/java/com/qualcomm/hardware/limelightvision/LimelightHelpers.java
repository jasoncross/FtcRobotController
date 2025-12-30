package com.qualcomm.hardware.limelightvision;

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
 *
 * CHANGES (2025-12-29): Added LimelightHelpers wrapper used by Odometry to
 *                        feed MT2 yaw and goal-only localization filters.
 */
public final class LimelightHelpers {
    private LimelightHelpers() {}

    private static final Map<String, Limelight3A> LIMELIGHTS = new ConcurrentHashMap<>();

    public static void registerLimelight(String name, Limelight3A limelight) {
        if (name == null || limelight == null) return;
        LIMELIGHTS.put(name, limelight);
    }

    public static void setRobotOrientation(String name,
                                           double yawDeg,
                                           double yawRateDegPerSec,
                                           double pitchDeg,
                                           double pitchRateDegPerSec,
                                           double rollDeg,
                                           double rollRateDegPerSec) {
        Limelight3A limelight = LIMELIGHTS.get(name);
        if (limelight == null) return;
        limelight.setRobotOrientation(
                yawDeg,
                yawRateDegPerSec,
                pitchDeg,
                pitchRateDegPerSec,
                rollDeg,
                rollRateDegPerSec
        );
    }

    public static void setFiducialIDFilters(String name, int[] ids) {
        Limelight3A limelight = LIMELIGHTS.get(name);
        if (limelight == null || ids == null) return;
        limelight.setFiducialIDFilters(ids);
    }
}
