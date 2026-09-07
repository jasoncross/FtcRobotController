package org.firstinspires.ftc.teamcode.drive;

import org.junit.Test;
import static org.junit.Assert.*;

public class DriveCoordinatesTest {
    @Test public void fieldForwardBecomesRobotRightWhenRobotFacesLeft() {
        assertArrayEquals(new double[]{0, 1}, MecanumMixer.fieldVector(0, 90), 1e-9);
        assertArrayEquals(new double[]{1, 0}, MecanumMixer.fieldVector(90, -90), 1e-9);
        assertArrayEquals(new double[]{-1, 0}, MecanumMixer.fieldVector(0, 180), 1e-9);
    }
    @Test public void slowingScalesNormalizedCombinedMotion() {
        double[] normal = MecanumMixer.mix(1, 1, 1, 0.4);
        double[] slow = MecanumMixer.mix(1, 1, 1, 0.4 * 0.25);
        for (int i = 0; i < 4; i++) assertEquals(normal[i] * 0.25, slow[i], 1e-9);
    }
}
