package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.config.TagAimTuning;
import org.firstinspires.ftc.teamcode.config.VisionConfig;

/** Retains the previous frame-delta PD tuning, clamp, and deadband with fresh-target gating. */
public final class TagAimController {
    private double lastError, lastPower;
    private long lastFrame;
    public void reset() { lastError = 0; lastPower = 0; lastFrame = 0; }
    public double turnPower(TargetObservation target, long nowNanos) {
        if (target == null || !target.isFresh(nowNanos, VisionConfig.MAX_TARGET_AGE_MS)
                || !Double.isFinite(target.bearingDeg)) {
            reset();
            return 0;
        }
        if (target.timestampNanos == lastFrame) return lastPower;
        double error = target.bearingDeg - VisionConfig.AIM_BEARING_OFFSET_DEG;
        double derivative = lastFrame == 0 ? 0 : error - lastError;
        lastError = error;
        lastFrame = target.timestampNanos;
        double power = TagAimTuning.KP * error + TagAimTuning.KD * derivative;
        double cap = Math.min(1, Math.abs(TagAimTuning.CLAMP_ABS));
        lastPower = Math.abs(error) <= Math.abs(TagAimTuning.DEADBAND_DEG)
                ? 0 : Math.max(-cap, Math.min(cap, power));
        return lastPower;
    }
}
