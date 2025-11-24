package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;
/*
 * FILE: Auto_Blue_30.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Provide a dead-simple BLUE alliance autonomous that just clears the wall:
 *     drive straight forward 30" at a gentle pace and stop. Useful for league
 *     events that only require launch-line movement points or quick partner
 *     coordination checks.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - move(... 30 in, heading 0°, twist 0°, speed 0.35)
 *
 * METHODS
 *   - alliance()
 *       • Identifies BLUE so BaseAuto mirrors telemetry colors and defaults.
 *   - startPoseDescription()
 *       • Quick reminder for drive teams that any along-the-wall placement is ok.
 *   - runSequence()
 *       • Issues the single move step and halts—nothing else is staged.
 *
 * NOTES
 *   - Adjust the distance or speed literal if events request a different motion
 *     requirement; SharedRobotTuning caps still apply for safety.
 */
@Autonomous(name="Auto: Blue 30", group="Auto", preselectTeleOp="TeleOp - Blue")
public class Auto_Blue_30 extends BaseAuto {
    // Alliance identity for BaseAuto scaffolding.
    @Override protected Alliance alliance() { return Alliance.BLUE; }
    // Telemetry label describing the expected robot orientation at init (edit
    // this string whenever the start pose changes so the Driver Station prompt
    // stays accurate).
    @Override protected String startPoseDescription() { return "Start: Blue Team — Anywhere along wall"; }

    // Primary autonomous path: drive 30" and stop.
    // CHANGES (2025-11-13): Documented minimalist 30" move behavior in header and removed unused imports.
    // CHANGES (2025-11-24): Added explicit twist parameter (0°) to AutoSequence.move(...) per new API.
    @Override
    protected void runSequence() throws InterruptedException {
        sequence()
                .move("Clear wall (drive 30 in)", 30.0, 0.0, 0.0, 0.35)
                .run();
    }
}
