package org.firstinspires.ftc.teamcode.vision;

import org.junit.Test;
import static org.junit.Assert.*;

public class VisionControlsTest {
    @Test public void selectorIgnoresSettlingRepeatedAndWrongPipelineFrames() {
        LimelightPipelineSelector selector = new LimelightPipelineSelector(new int[]{0, 1}, 2, 2, 100, 10, 1000, 3, 0);
        selector.update(50, 1, 0, true); // Not settled.
        selector.update(100, 2, 1, true); // Old/wrong pipeline.
        selector.update(110, 3, 0, true);
        selector.update(130, 3, 0, true); // Same frame must not count twice.
        assertEquals(0, selector.currentPipeline());
        selector.update(150, 4, 0, true);
        assertEquals(1, selector.currentPipeline());
        selector.update(250, 5, 1, false);
        selector.update(270, 6, 1, true);
        assertTrue(selector.isComplete());
        assertEquals(0, selector.currentPipeline()); // Only pipeline 0 had two hits.
    }

    @Test public void selectorTimesOutWithoutFramesAndUsesFallback() {
        LimelightPipelineSelector selector = new LimelightPipelineSelector(new int[]{1}, 2, 2, 100, 10, 500, 0, 0);
        selector.update(500, Long.MIN_VALUE, -1, false);
        assertTrue(selector.isComplete());
        assertEquals(0, selector.currentPipeline());
    }

    @Test public void failedPipelineUsesFallbackEvenWithCompleteSampling() {
        LimelightPipelineSelector selector = new LimelightPipelineSelector(new int[]{1}, 2, 2, 0, 0, 500, 0, 0);
        selector.update(1, 1, 1, false);
        selector.update(2, 2, 1, true);
        assertTrue(selector.isComplete());
        assertEquals(0, selector.currentPipeline());
    }

    @Test public void aimHasCorrectSignClampDeadbandAndNoDerivativeKickAfterLoss() {
        TagAimController aim = new TagAimController();
        long now = 1_000_000_000L;
        assertEquals(0.2, aim.turnPower(new TargetObservation(5, 10, 1, now), now), 1e-9);
        assertEquals(0.2, aim.turnPower(new TargetObservation(5, 10, 1, now), now + 1), 1e-9);
        assertEquals(0, aim.turnPower(null, now), 0);
        assertEquals(-0.2, aim.turnPower(new TargetObservation(5, -10, 1, now + 1), now + 1), 1e-9);
        assertEquals(0.6, aim.turnPower(new TargetObservation(5, 100, 1, now + 2), now + 2), 1e-9);
        assertEquals(0, aim.turnPower(new TargetObservation(5, 1, 1, now + 3), now + 3), 0);
    }

    @Test public void aimStopsForStaleFutureAndUnsolvedObservations() {
        TagAimController aim = new TagAimController();
        long now = 1_000_000_000L;
        assertEquals(0, aim.turnPower(new TargetObservation(5, 10, 1, now), now + 121_000_000L), 0);
        assertEquals(0, aim.turnPower(new TargetObservation(5, 10, 1, now + 1), now), 0);
        assertEquals(0, aim.turnPower(new TargetObservation(5, Double.NaN, Double.NaN, now), now), 0);
    }
}
