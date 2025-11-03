package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.auto.BaseAuto.ScanDirection;

/*
 * FILE: Auto_Blue_Human.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Run the BLUE alliance human-player side start (west tile, robot facing
 *     NORTH) exactly as described in DECODE_Season_Context.md so the preload is
 *     launched before transitioning toward the classifier zone.
 *   - Demonstrate how the AutoSequence builder stitches together the shared
 *     scan, aim, fire, and drive helpers for a straightforward "find Tag 20 →
 *     shoot → drive" routine that students can iterate on quickly.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - move(... 2 in, heading 0°, speed 0.35)
 *       • Gentle wall-clear bump before the tag scan.
 *   - rotateToTarget(label, ScanDirection.CCW, turnSpeed 0.25, sweep 90°/30°)
 *       • Sweeps counter-clockwise up to 90° after the wall-clear, then checks
 *         30° clockwise while searching for Tag 20. Increase the sweep angles
 *         or speed fraction for more aggressive hunts.
 *   - aim(timeout 3200 ms)
 *       • Waits for the launcher to reach SharedRobotTuning.RPM_TOLERANCE.
 *   - fire(shots = 3, betweenShotsMs = 3000)
 *       • Supplies cadence inline so each sequence controls its own spacing.
 *   - move(... 24 in, heading 0°, speed 0.55)
 *       • Distance toward the classifier once the preload volley is finished.
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
@Autonomous(name="Auto: Blue Human", group="Auto", preselectTeleOp="TeleOp - Blue")
public class Auto_Blue_Human extends BaseAuto {
    // CHANGES (2025-10-31): Added wall-clear drive, telemetry-guided tag scan, locked volley,
    //                        heading reset, and 24" advance per refreshed Auto steps.
    // CHANGES (2025-10-31): Switched to AutoSequence for clearer movement/aim/fire scripting
    //                        with adjustable power caps and explicit tag scan control.
    // CHANGES (2025-11-02): Added AutoSpeed pre-spin warm-up and configurable volley spacing parameter.
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
                .rememberHeading("Record start heading")
                .move("Clear wall (drive 2 in)", 2.0, 0.0, 0.35)
                .spinToAutoRpm("Pre-spin launcher to auto RPM")
                .rotateToTarget("Scan for Tag 20", ScanDirection.CCW, 0.25, 90, 30)
                .aim("Spin launcher for volley", 3200)
                .fire("Fire 3-shot volley", 3, true, 3000)
                .returnToStoredHeading("Return to start heading", 0.45)
                .move("Drive 24 in upfield", 24.0, 0.0, 0.55)
                .run();
    }
}
