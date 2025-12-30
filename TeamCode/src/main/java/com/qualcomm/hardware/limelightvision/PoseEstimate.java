package com.qualcomm.hardware.limelightvision;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class PoseEstimate {
    public final Pose3D pose;
    public final double latencyMs;
    public final int tagCount;
    public final double timestampSeconds;

    public PoseEstimate(Pose3D pose, double latencyMs) {
        this(pose, latencyMs, -1, Double.NaN);
    }

    public PoseEstimate(Pose3D pose, double latencyMs, int tagCount, double timestampSeconds) {
        this.pose = pose;
        this.latencyMs = latencyMs;
        this.tagCount = tagCount;
        this.timestampSeconds = timestampSeconds;
    }
}
