package org.firstinspires.ftc.teamcode.input;

import org.firstinspires.ftc.teamcode.config.ControllerTuning;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Tests the actual trigger mapping used by BaseDriveTeleOp.
 * CHANGES (2026-09-12): Cover trigger endpoints, intermediate braking and configured floor clamps.
 */
public class DriveInputScalingTest {
    @Test public void triggerTravelAndBoundsUseConfiguredFloor() {
        double saved = ControllerTuning.SLOWEST_SPEED;
        try {
            ControllerTuning.SLOWEST_SPEED = 0.25;
            assertEquals(1, DriveInputScaling.slowScale(0), 0);
            assertEquals(0.625, DriveInputScaling.slowScale(0.5), 0);
            assertEquals(0.25, DriveInputScaling.slowScale(1), 0);
            assertEquals(1, DriveInputScaling.slowScale(-0.4), 0);
            assertEquals(0.25, DriveInputScaling.slowScale(1.4), 0);
            ControllerTuning.SLOWEST_SPEED = 0.6;
            assertEquals(0.8, DriveInputScaling.slowScale(0.5), 1e-12);
            ControllerTuning.SLOWEST_SPEED = -0.5;
            assertEquals(0, DriveInputScaling.slowScale(1), 0);
            ControllerTuning.SLOWEST_SPEED = 1.5;
            assertEquals(1, DriveInputScaling.slowScale(1), 0);
        } finally {
            ControllerTuning.SLOWEST_SPEED = saved;
        }
    }
}
