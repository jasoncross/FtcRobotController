package org.firstinspires.ftc.teamcode.input;

import org.firstinspires.ftc.teamcode.config.ControllerTuning;

/**
 * Driver trigger braking, shared by TeleOp and its regression tests.
 * CHANGES (2026-09-12): Extract the existing trigger-to-speed calculation without changing defaults.
 */
public final class DriveInputScaling {
    private DriveInputScaling() { }

    /** Returns a fraction of the drivetrain power cap, reading the current configured floor. */
    public static double slowScale(double trigger) {
        double floor = Math.max(0, Math.min(1, ControllerTuning.SLOWEST_SPEED));
        return 1 - Math.max(0, Math.min(1, trigger)) * (1 - floor);
    }
}
