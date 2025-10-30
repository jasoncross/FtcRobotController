/*
 * FILE: IntakeTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Centralize intake motor power so TeleOp and Auto stay aligned when hardware
 *     or battery behavior changes.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Intake power & driver defaults)
 *   - POWER_ON
 *       • Motor power commanded while the intake runs. Increase toward 0.9 when
 *         ARTIFACTS slip; reduce toward 0.6 if jams appear.
 */
package org.firstinspires.ftc.teamcode.config;

public final class IntakeTuning {
    private IntakeTuning() {}

    public static double POWER_ON = 0.8;
}
