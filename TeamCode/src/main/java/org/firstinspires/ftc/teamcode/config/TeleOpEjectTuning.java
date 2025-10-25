/*
 * FILE: TeleOpEjectTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Keep the TeleOp-only eject routine parameters—temporary launcher RPM and
 *     duration—in one place so clearing jams can be retuned quickly.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Shot cadence, feed, and eject)
 *   - RPM
 *       • Launcher RPM commanded during the eject sequence. Stays clamped by
 *         LauncherTuning.RPM_MIN/RPM_MAX downstream.
 *   - TIME_MS
 *       • Duration of the eject routine before the prior RPM is restored.
 */
package org.firstinspires.ftc.teamcode.config;

public final class TeleOpEjectTuning {
    private TeleOpEjectTuning() {}

    public static double RPM    = 600.0;
    public static int    TIME_MS = 1000;
}
