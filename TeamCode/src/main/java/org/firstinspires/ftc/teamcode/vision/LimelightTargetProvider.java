package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.config.LimelightTuning;
import org.firstinspires.ftc.teamcode.config.VisionConfig;

/** Native FTC Limelight 3A polling and target selection. No NetworkTables or field-pose fusion. */
public final class LimelightTargetProvider implements VisionTargetProvider {
    private final Limelight3A limelight;
    private final LimelightPipelineSelector selector;
    private int pipeline;
    private long lastFrame = Long.MIN_VALUE;
    private List<TargetObservation> targets = Collections.emptyList();

    public LimelightTargetProvider(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, VisionConfig.LIMELIGHT_NAME);
        long now = System.nanoTime() / 1_000_000;
        selector = LimelightTuning.AUTO_SELECT ? new LimelightPipelineSelector(
                LimelightTuning.PIPELINES, LimelightTuning.SAMPLE_COUNT, LimelightTuning.MIN_TARGET_HITS,
                LimelightTuning.SETTLE_MS, LimelightTuning.SAMPLE_INTERVAL_MS,
                LimelightTuning.MAX_SELECTION_MS, LimelightTuning.FALLBACK_INDEX, now) : null;
        pipeline = selector == null ? LimelightTuning.PIPELINE_INDEX : selector.currentPipeline();
        try {
            limelight.setPollRateHz(LimelightTuning.POLL_HZ);
            switchPipeline(pipeline);
            limelight.start();
        } catch (RuntimeException e) {
            limelight.stop();
            throw e;
        }
    }

    private void switchPipeline(int next) {
        if (!limelight.pipelineSwitch(next)) throw new IllegalStateException("Cannot select Limelight pipeline " + next);
        pipeline = next;
        targets = Collections.emptyList();
        lastFrame = Long.MIN_VALUE;
    }

    @Override public void update() {
        LLResult result = limelight.getLatestResult();
        boolean fresh = result != null && result.getStaleness() <= VisionConfig.MAX_TARGET_AGE_MS;
        boolean matches = fresh && result.getPipelineIndex() == pipeline;
        long frame = matches ? result.getControlHubTimeStampNanos() : Long.MIN_VALUE;
        boolean qualifying = false;
        if (matches && result.isValid()) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (qualifies(tag.getFiducialId())) qualifying = true;
            }
        }
        if (selector != null && !selector.isComplete()) {
            selector.update(System.nanoTime() / 1_000_000, frame,
                    matches ? result.getPipelineIndex() : -1, qualifying);
            if (selector.currentPipeline() != pipeline) {
                switchPipeline(selector.currentPipeline());
                return;
            }
        }
        if (!matches || !result.isValid()) { targets = Collections.emptyList(); return; }
        if (frame == lastFrame) return;
        lastFrame = frame;
        long acquired = System.nanoTime() - result.getStaleness() * 1_000_000;
        List<TargetObservation> found = new ArrayList<>();
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            // Bearing is useful without trusting the previous robot's field map or camera transform.
            found.add(new TargetObservation(tag.getFiducialId(), tag.getTargetXDegrees(), Double.NaN, acquired));
        }
        targets = found;
    }

    private boolean qualifies(int id) {
        if (LimelightTuning.QUALIFYING_TAG_IDS.length == 0) return true;
        for (int allowed : LimelightTuning.QUALIFYING_TAG_IDS) if (id == allowed) return true;
        return false;
    }

    /** Explicit yaw feed for future MT2 work; caller must verify frame/mounting convention. */
    public boolean updateRobotYaw(double yawDeg) {
        return Double.isFinite(yawDeg) && limelight.updateRobotOrientation(yawDeg);
    }

    @Override public List<TargetObservation> getTargets() {
        List<TargetObservation> fresh = new ArrayList<>();
        long now = System.nanoTime();
        for (TargetObservation target : targets) if (target.isFresh(now, VisionConfig.MAX_TARGET_AGE_MS)) fresh.add(target);
        return fresh;
    }
    @Override public String getStatus() {
        return "Limelight " + (limelight.isConnected() ? "connected" : "disconnected") + " pipeline=" + pipeline
                + (selector != null && !selector.isComplete() ? " (selecting)" : "");
    }
    @Override public void close() { limelight.stop(); }
}
