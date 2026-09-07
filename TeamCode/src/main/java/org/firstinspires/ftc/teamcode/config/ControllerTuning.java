/** Driver input thresholds and scaling. Trigger threshold retained from DECODE. */
package org.firstinspires.ftc.teamcode.config;

public final class ControllerTuning {
    private ControllerTuning() {}

    public static double TRIGGER_EDGE_THRESH = 0.5; // Trigger value treated as a press when edge-detected
    public static double STICK_DEADBAND = 0.05;
    public static double SLOWEST_SPEED = 0.25; // Retained trigger-brake floor (fraction of configured cap)
    public static double STRAFE_SCALE = 1.0;
    public static double TURN_SCALE = 1.0;
}
