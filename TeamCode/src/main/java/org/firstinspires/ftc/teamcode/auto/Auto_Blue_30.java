package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.config.VisionTuning;

/*
 * FILE: Auto_Blue_30.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Run the BLUE alliance robot 30 inches forward and stops.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - move(... 30 in, heading 0°, speed 0.35)
 *
 * METHODS
 *   - alliance()
 *       • Identifies the BLUE alliance so BaseAuto selects Tag 20 by default.
 *   - startPoseDescription()
 *       • Telemetry helper used by drive teams to confirm the robot is staged
 *         correctly before autonomous starts.
 *   - runSequence()
 *       • Main autonomous choreography: capture heading, aim, wait for RPM,
 *         fire, and reposition.
 *
 * NOTES
 *   - Any tuning to SharedRobotTuning propagates here immediately; only adjust
 *     the literal values in this file when the BLUE human route itself changes.
 *   - If TeleOpAllianceBase overrides AutoAim defaults (e.g., initialAutoDefaultSpeed),
 *     ensure BaseAuto.syncTeleOpOverrides() still mirrors those settings at init.
 */
@Autonomous(name="Auto: Blue 30", group="Auto", preselectTeleOp="TeleOp - Blue")
public class Auto_Blue_30 extends BaseAuto {
    // Alliance identity for BaseAuto scaffolding.
    @Override protected Alliance alliance() { return Alliance.BLUE; }
    // Telemetry label describing the expected robot orientation at init (edit
    // this string whenever the start pose changes so the Driver Station prompt
    // stays accurate).
    @Override protected String startPoseDescription() { return "Start: Blue Team — Anywhere along wall"; }

    // Primary autonomous path: turn, confirm RPM, fire preload, reposition.
    @Override
    protected void runSequence() throws InterruptedException {
        sequence()
                .move("Clear wall (drive 30 in)", 30.0, 0.0, 0.35)
                .run();
    }
}
