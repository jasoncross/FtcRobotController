package org.firstinspires.ftc.teamcode.vision;

/** Nonblocking pipeline sampling; only distinct frames from the requested pipeline count. */
public final class LimelightPipelineSelector {
    private final int[] pipelines;
    private final int sampleCount, minimumHits, fallback;
    private final long settleMs, intervalMs, timeoutMs;
    private int index, samples, hits, bestHits = -1, bestPipeline = -1;
    private long startedMs, switchedMs, sampledMs, lastFrame = Long.MIN_VALUE;
    private boolean complete;

    public LimelightPipelineSelector(int[] pipelines, int sampleCount, int minimumHits,
            long settleMs, long intervalMs, long timeoutMs, int fallback, long nowMs) {
        if (pipelines.length == 0 || sampleCount <= 0 || minimumHits < 1 || minimumHits > sampleCount
                || settleMs < 0 || intervalMs < 0 || timeoutMs <= 0) {
            throw new IllegalArgumentException("Invalid pipeline sampling settings");
        }
        this.pipelines = pipelines.clone(); this.sampleCount = sampleCount;
        this.minimumHits = minimumHits; this.settleMs = settleMs;
        this.intervalMs = intervalMs; this.timeoutMs = timeoutMs; this.fallback = fallback;
        startedMs = switchedMs = sampledMs = nowMs;
    }

    public int currentPipeline() {
        return complete ? (bestPipeline >= 0 ? bestPipeline : fallback) : pipelines[index];
    }
    public boolean isComplete() { return complete; }

    /** No reset on START: this selection has a bounded lifetime beginning when initiated. */
    public void update(long nowMs, long frameId, int framePipeline, boolean qualifying) {
        if (complete) return;
        if (nowMs - startedMs >= timeoutMs) { complete = true; return; }
        if (nowMs - switchedMs < settleMs || nowMs - sampledMs < intervalMs
                || frameId == Long.MIN_VALUE || frameId == lastFrame || framePipeline != pipelines[index]) return;
        lastFrame = frameId;
        sampledMs = nowMs;
        samples++;
        if (qualifying) hits++;
        if (samples < sampleCount) return;
        if (hits >= minimumHits && hits > bestHits) {
            bestHits = hits;
            bestPipeline = pipelines[index];
        }
        index++;
        if (index == pipelines.length) { complete = true; return; }
        switchedMs = nowMs;
        hits = samples = 0;
    }
}
