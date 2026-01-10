package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.auto.BaseAuto.ScanDirection;
import org.firstinspires.ftc.teamcode.config.VisionTuning;

/*
 * FILE: Auto_Red_Human_LongShot.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Execute the RED alliance human-player start with a launch-line long shot:
 *     slide 3" off the wall, scan for Tag 24 without leaving the launch tile,
 *     fire a five-artifact volley, then drive 36" forward to stage for TeleOp
 *     cycles.
 *   - Mirror the BLUE long-shot variant so both alliances document the minimal
 *     movement route alongside the full-field sprint option.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - visionMode(... Mode.P720)
 *       • Enables the 720p sighting profile before the aim so range sampling
 *         mirrors TeleOp’s long-shot configuration.
 *   - move(... 3 in, heading 0°, twist 0°, speed 0.35)
 *       • Clears the wall contact before aiming to avoid rubbing during the
 *         stationary volley while holding the starting orientation.
 *   - rotateToTarget(label, ScanDirection.CW, turnSpeed 0.25, sweep 90°/30°, timeout 10000 ms)
 *       • Uses the mirrored clockwise sweep envelope to hunt Tag 24 while
 *         staying on the launch tile; adjust timeout/angles as strategy changes.
 *   - spinToAutoRpmDefault(...)
 *       • Keeps the launcher at the AutoSpeed standby RPM while staged on the
 *         launch tile.
 *   - readyToLaunch(timeout 3200 ms)
 *       • Waits for AutoSpeed to satisfy the shared RPM window + settle timer
 *         so every shot leaves at target velocity.
 *   - fire(shots = 5, betweenShotsMs = 1000)
 *       • Matches the five-artifact volley timing used on BLUE for parity across
 *         alliances.
 *   - move(... 36 in, heading 0°, twist 0°, speed 0.85)
 *       • Drives 36" upfield post-volley to open the teleop lane immediately.
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
@Autonomous(name="Auto: Red Human Long Shot", group="Auto", preselectTeleOp="TeleOp - Red")
public class Auto_Red_Human_LongShot extends BaseAuto {
    // CHANGES (2025-10-31): Added wall-clear bump, telemetry-guided Tag 24 volley, heading
    //                        reset, and 24" advance mirroring the refreshed Auto flow.
    // CHANGES (2025-10-31): Migrated to AutoSequence builder for declarative steps and
    //                        clarified lock/aim/fire sequencing with configurable speed caps.
    // CHANGES (2025-11-02): Added AutoSpeed warm-up stage and explicit volley spacing parameter.
    // CHANGES (2025-11-03): Renamed launcher prep steps to readyToLaunch()/spinToAutoRpmDefault() and
    //                        adopted the shared AutoSpeed settle behavior.
    // CHANGES (2025-11-05): Added 720p vision profile swap at sequence start to mirror TeleOp testing.
    // CHANGES (2025-11-13): Corrected header for long-shot variant (3" slide, five-shot volley, 36" advance).
    // CHANGES (2025-11-25): rotateToTarget scan now uses an inline 10 s timeout instead of the BaseAuto default field.
    // CHANGES (2025-11-26): Standardized rotate-to-target timeout literal to 10000 ms for readability.
    // CHANGES (2025-11-24): Added explicit twist parameters (0°) to AutoSequence.move(...) calls per new API.
    // CHANGES (2025-12-11): Recentered odometry start pose to (+12, -72, 0) in the field-center frame (human wall = −72" Y).
    // CHANGES (2026-01-09): Added a 1s endgame reserve and moved the final retreat drive into ENDGAME sequencing.
    // CHANGES (2026-01-10): Added AutoSequence AutoRPM tweak step for TeleOp-style scale adjustments.
    private static final long ENDGAME_RESERVE_MS = 1500;
    // Provide BaseAuto the active alliance to load correct AprilTag data.
    @Override protected Alliance alliance() { return Alliance.RED; }
    @Override protected long endgameReserveMs() { return ENDGAME_RESERVE_MS; }
    public Auto_Red_Human_LongShot() { setStartingPose(12.0, -63.0, 0.0); }
    // Telemetry callout for the field-side volunteer verifying orientation (edit
    // this whenever start staging changes so the Start Pose telemetry stays
    // correct).
    @Override protected String startPoseDescription() { return "Start: Red Human — East of south firing triangle, FACING NORTH"; }

    // Main autonomous path: aim, fire, and roll forward for cycle setup.
    @Override
    protected void runSequence() throws InterruptedException {
        sequence()
                .adjustAutoScale("AutoRPM tweak +2%", 0.02)
                //.visionMode("Switch to 480p vision", VisionTuning.Mode.P480)
                .rememberHeading("Record start heading")
                .move("Drive forward to clear wall", 11.0, 0.0, 0.0, 1)
                .rotateToTarget("Scan for Tag", ScanDirection.CW, 0.4, -30, 30, 10000)
                //.readyToLaunch("Ready launcher for volley", 500)
                .readyToLaunch("Ready launcher for volley", 500, 119)
                .fireContinuous("firing",2000,true, true)
                .returnToStoredHeading("Return to start heading", 0.85)
                .move("Drive to balls", 20.0, -45.0, 90.0, 1)
                .move("Drive backward", 37.0, 180.0, 0.0, 1)
                .move("Drive to triangle", 53.0, 27.0, -115.0, 1.0)
                .rotateToTarget("Scan for Tag", ScanDirection.CW, 0.4, -30, 30, 1000)
                .readyToLaunch("Ready launcher for volley", 500, 114)
                .fireContinuous("firing",1500,false, true)
                .endgameMove("Drive out",8,0,0,1)
          .run();
    }
}
