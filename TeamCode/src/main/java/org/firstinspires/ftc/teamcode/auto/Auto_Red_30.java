package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;
/*
 * FILE: Auto_Red_30.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Deliver the mirrored RED alliance autonomous that only needs movement
 *     points: drive straight forward 30" from the launch wall at a modest pace
 *     and stop.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - move(... 30 in, heading 0°, speed 0.35)
 *
 * METHODS
 *   - alliance()
 *       • Flags RED so BaseAuto keeps telemetry + goal references consistent.
 *   - startPoseDescription()
 *       • Reminds setup crew any wall-aligned staging works for the safety move.
 *   - runSequence()
 *       • Queues the single move and exits; no launcher actions are requested.
 *
 * NOTES
 *   - Tweak the distance or speed literal if event policies change; drive caps
 *     from SharedRobotTuning still gate the final power.
 */
@Autonomous(name="Auto: Red 30", group="Auto", preselectTeleOp="TeleOp - Red")
public class Auto_Red_30 extends BaseAuto {
    // Provide BaseAuto the active alliance to load correct AprilTag data.
    @Override protected Alliance alliance() { return Alliance.RED; }
    // Telemetry callout for the field-side volunteer verifying orientation (edit
    // this whenever start staging changes so the Start Pose telemetry stays
    // correct).
    @Override protected String startPoseDescription() { return "Start: Red Team — Anywhere along wall"; }

    // Main autonomous path: drive 30" and stop.
    // CHANGES (2025-11-13): Updated header to describe pure-movement plan and removed unused imports.
    @Override
    protected void runSequence() throws InterruptedException {
        sequence()
                .move("Clear wall (drive 30 in)", 30.0, 0.0, 0.35)
                .run();
    }
}
