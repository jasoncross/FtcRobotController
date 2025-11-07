package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.config.VisionTuning;

/*
 * FILE: Auto_Red_30.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Move the RED alliance robot 30 inches forward and stops.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - move(... 30 in, heading 0°, speed 0.35)
 *
 * METHODS
 *   - alliance()
 *       • Identifies RED so BaseAuto requests Tag 24 and mirrors field geometry.
 *   - startPoseDescription()
 *       • Telemetry text ensuring the field crew stages the robot correctly.
 *   - runSequence()
 *       • Captures the starting heading, performs the aim/spin checks, fires,
 *         then re-aligns and drives forward.
 *
 * NOTES
 *   - SharedRobotTuning updates propagate here automatically; alter the literal
 *     values only when the RED human strategy changes (e.g., different drive
 *     distance or follow-up task).
 */
@Autonomous(name="Auto: Red 30", group="Auto", preselectTeleOp="TeleOp - Red")
public class Auto_Red_30 extends BaseAuto {
    // Provide BaseAuto the active alliance to load correct AprilTag data.
    @Override protected Alliance alliance() { return Alliance.RED; }
    // Telemetry callout for the field-side volunteer verifying orientation (edit
    // this whenever start staging changes so the Start Pose telemetry stays
    // correct).
    @Override protected String startPoseDescription() { return "Start: Red Team — Anywhere along wall"; }

    // Main autonomous path: aim, fire, and roll forward for cycle setup.
    @Override
    protected void runSequence() throws InterruptedException {
        sequence()
                .move("Clear wall (drive 30 in)", 30.0, 0.0, 0.35)
                .run();
    }
}
