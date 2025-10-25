/*
 * FILE: ControllerTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Expose controller-level thresholds that affect how trigger presses are
 *     interpreted so students can tweak input feel without modifying the binding
 *     system.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Controller interface & misc utilities)
 *   - TRIGGER_EDGE_THRESH
 *       • Analog trigger value treated as a "pressed" button when using
 *         bindTriggerPress. Lower for lighter pulls; raise when drivers bump
 *         triggers accidentally.
 */
package org.firstinspires.ftc.teamcode.config;

public final class ControllerTuning {
    private ControllerTuning() {}

    public static double TRIGGER_EDGE_THRESH = 0.5;
}
