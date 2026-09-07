package org.firstinspires.ftc.teamcode.vision;

/** One camera observation. Bearing is +right/CW; distance is camera-to-tag in meters. */
public final class TargetObservation {
    public final int id;
    public final double bearingDeg, distanceMeters;
    public final long timestampNanos;
    public TargetObservation(int id, double bearingDeg, double distanceMeters, long timestampNanos) {
        this.id = id; this.bearingDeg = bearingDeg;
        this.distanceMeters = distanceMeters; this.timestampNanos = timestampNanos;
    }
    public boolean isFresh(long nowNanos, long maxAgeMs) {
        long age = nowNanos - timestampNanos;
        return timestampNanos > 0 && age >= 0 && age / 1_000_000.0 <= maxAgeMs;
    }
}
