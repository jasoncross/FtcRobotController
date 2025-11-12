package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.auto.BaseAuto.ScanDirection;
import org.firstinspires.ftc.teamcode.config.VisionTuning;

/*
 * FILE: Auto_Blue_Human_LongShot.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Execute the BLUE alliance human-player start with a stationary long-shot
 *     focus: bump 3" off the wall, sweep for Tag 20 from the launch line,
 *     unleash a five-artifact volley, then push 36" upfield toward the
 *     classifier lane for teleop pickup.
 *   - Provide a contrast to the full-field sprint auto by documenting the
 *     minimal-movement variant that clears the wall, fires immediately, and then
 *     drives forward for cycle staging.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - visionMode(... Mode.P720)
 *       • Swaps into the 720p sighting profile before aiming to maximize range
 *         resolution from the launch line.
 *   - move(... 3 in, heading 0°, speed 0.35)
 *       • Slides off the wall just enough to remove bumper pressure before the
 *         tag scan.
 *   - rotateToTarget(label, ScanDirection.CCW, turnSpeed 0.25, sweep 90°/30°)
 *       • Maintains the standard CCW sweep envelope while aiming from the launch
 *         line; tune angles or speed for alternate search coverage.
 *   - spinToAutoRpmDefault(...)
 *       • Keeps the launcher warm with the shared AutoSpeed standby RPM while
 *         the robot remains near the wall.
 *   - readyToLaunch(timeout 3200 ms)
 *       • Holds until AutoSpeed reaches the shared RPM window and settle timer,
 *         guaranteeing consistency before the long volley.
 *   - fire(shots = 5, betweenShotsMs = 1000)
 *       • Fires the same five-artifact volley used in the long-run auto, holding
 *         1 s between shots for recovery.
 *   - move(... 36 in, heading 0°, speed 0.85)
 *       • Drives 36" upfield after shooting to stage closer to the classifier
 *         lane for rapid TeleOp cycles.
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
@Autonomous(name="Auto: Blue Human Long Shot", group="Auto", preselectTeleOp="TeleOp - Blue")
public class Auto_Blue_Human_LongShot extends BaseAuto {
    // CHANGES (2025-10-31): Added wall-clear drive, telemetry-guided tag scan, locked volley,
    //                        heading reset, and 24" advance per refreshed Auto steps.
    // CHANGES (2025-10-31): Switched to AutoSequence for clearer movement/aim/fire scripting
    //                        with adjustable power caps and explicit tag scan control.
    // CHANGES (2025-11-02): Added AutoSpeed pre-spin warm-up and configurable volley spacing parameter.
    // CHANGES (2025-11-03): Renamed launcher prep steps to readyToLaunch()/spinToAutoRpmDefault() and
    //                        adopted the shared AutoSpeed settle behavior.
    // CHANGES (2025-11-05): Added 720p vision profile swap at sequence start to mirror TeleOp testing.
    // CHANGES (2025-11-13): Corrected header for long-shot variant (3" bump, five-shot volley, 36" advance).
    // Alliance identity for BaseAuto scaffolding.
    @Override protected Alliance alliance() { return Alliance.BLUE; }
    // Telemetry label describing the expected robot orientation at init (edit
    // this string whenever the start pose changes so the Driver Station prompt
    // stays accurate).
    @Override protected String startPoseDescription() { return "Start: Blue Human — West of south firing triangle, FACING NORTH"; }

    // Primary autonomous path: turn, confirm RPM, fire preload, reposition.
    @Override
    protected void runSequence() throws InterruptedException {
        sequence()
                .visionMode("Switch to 720p vision", VisionTuning.Mode.P720)
                .rememberHeading("Record start heading")
                .spinToAutoRpmDefault("Pre-spin launcher to auto RPM")
                .move("Drive forward 3 inches to clear wall", 3.0, 0.0, 0.35)
                .rotateToTarget("Scan for Tag", ScanDirection.CCW, 0.25, 90, 30)
                .readyToLaunch("Ready launcher for volley", 3200)
                .fire("Fire volley", 5, true, 1000)
                .returnToStoredHeading("Return to start heading", 0.45)
                .move("Drive 36 in forward", 36.0, 0.0, 0.85)
                .run();
    }
}
