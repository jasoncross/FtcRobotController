package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.auto.BaseAuto.ScanDirection;
import org.firstinspires.ftc.teamcode.config.VisionTuning;

/*
 * FILE: Auto_Blue_Human.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Execute the BLUE alliance human-player start (west tile, robot facing
 *     NORTH) by sprinting upfield to the long-range launch point, sweeping for
 *     Tag 20, firing a five-artifact volley, then backpedaling to the south to
 *     reopen the alliance lane for partner cycles.
 *   - Demonstrate the extended AutoSequence pattern that pairs a vision profile
 *     swap, heading capture, aggressive upfield drive, and rapid-fire cadence
 *     so students can tune long-range routes without rewriting helpers.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - visionMode(... Mode.P720)
 *       • Switches the camera into the 720p sighting pipeline before leaving
 *         the wall so range sampling matches TeleOp long-shot tuning.
 *   - spinToAutoRpmDefault(...)
 *       • Keeps the launcher warm with the shared AutoSpeed default during the
 *         long drive to the firing spot.
 *   - move(... 80 in, heading 0°, twist 0°, speed 0.35)
 *       • Drives the full length to the calibrated long-shot standoff while
 *         holding the starting orientation before beginning the tag sweep.
 *   - rotateToTarget(label, ScanDirection.CCW, turnSpeed 0.25, sweep 90°/30°, timeout 10000 ms)
 *       • Sweeps counter-clockwise to 90° then checks 30° clockwise while
 *         hunting for Tag 20; adjust angles/speed for alternate scan envelopes and
 *         raise/lower the timeout per route.
 *   - readyToLaunch(timeout 3200 ms)
 *       • Waits for AutoSpeed to hit the shared RPM window + settle timer before
 *         allowing the volley to start.
 *   - fire(shots = 5, betweenShotsMs = 1000)
 *       • Commands a rapid five-artifact volley once RPM readiness is confirmed.
 *   - move(... -36 in, heading 0°, twist 0°, speed 0.85)
 *       • Retreats 36" toward the launch line to clear space for alliance
 *         partners after the long volley.
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
    // CHANGES (2025-11-03): Renamed launcher prep steps to readyToLaunch()/spinToAutoRpmDefault() and
    //                        adopted the shared AutoSpeed settle behavior.
    // CHANGES (2025-11-05): Added 720p vision profile swap at sequence start to mirror TeleOp testing.
    // CHANGES (2025-11-13): Updated header to match long-range drive/return plan and document five-shot cadence + retreat.
    // CHANGES (2025-11-25): Inlined the 10 s rotate-to-target timeout directly on the scan step instead of using BaseAuto's default.
    // CHANGES (2025-11-26): Standardized rotate-to-target timeout literal to 10000 ms for readability.
    // CHANGES (2025-11-24): Added explicit twist parameters (0°) to AutoSequence.move(...) calls per new API.
    // CHANGES (2025-12-11): Recentered odometry start pose to (-12, -72, 0) in the field-center frame (human wall = −72" Y).
    // CHANGES (2026-01-09): Added a 1s endgame reserve and moved the final retreat drive into ENDGAME sequencing.
    private static final long ENDGAME_RESERVE_MS = 1500;
    // Alliance identity for BaseAuto scaffolding.
    @Override protected Alliance alliance() { return Alliance.BLUE; }
    @Override protected long endgameReserveMs() { return ENDGAME_RESERVE_MS; }
    public Auto_Blue_Human() { setStartingPose(-12.0, -63.0, 0.0); }
    // Telemetry label describing the expected robot orientation at init (edit
    // this string whenever the start pose changes so the Driver Station prompt
    // stays accurate).
    @Override protected String startPoseDescription() { return "Start: Blue Human — West of south firing triangle, FACING NORTH"; }

    // Primary autonomous path: turn, confirm RPM, fire preload, reposition.
    @Override
    protected void runSequence() throws InterruptedException {
        sequence()
                //.visionMode("Switch to 480p vision", VisionTuning.Mode.P480)
                .rememberHeading("Record start heading")
                .move("Drive forward to target firing zone", 74.0, 0.0, 0.0, 1.0)
                .rotateToTarget("Scan for Tag", ScanDirection.CCW, 0.4, 90, 30, 10000)
                .readyToLaunch("Ready launcher for volley", 0, 62)
                .fireContinuous("firing",1500,true, false)
                //.fire("Fire volley", 3, true, true, 000)
                .returnToStoredHeading("Return to start heading", 0.85)
                .move("Drive to balls", 52.0, 170.0, -90.0, 1.0)
                .move("Drive backward", 34.0, 180.0, 0.0, 1.0)
                .move("Drive to triangle", 56.0, -26.0, 115.0, 1.0)
                .rotateToTarget("Scan for Tag", ScanDirection.CCW, 0.4, -30, 30, 1000)
                .readyToLaunch("Ready launcher for volley", 500,117 )
                .fireContinuous("firing",1500,false, true)
                .endgameMove("Drive out",8,0,0,1)
                .run();
    }
}
